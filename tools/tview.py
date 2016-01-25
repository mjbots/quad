#!/usr/bin/env python

# Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''%prog [options]

Interactively display and update values from an embedded device.
'''

import optparse
import os
import re
import serial
import struct
import sys
import time

import matplotlib

matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4'] = 'PySide'

from matplotlib.backends import backend_qt4agg
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

from IPython.external.qt import QtCore, QtGui
from IPython.qt.console.history_console_widget import HistoryConsoleWidget

SCRIPT_PATH=os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(SCRIPT_PATH, '../python'))
sys.path.append(os.path.join(SCRIPT_PATH, 'build-x86_64'))

import telemetry_archive
import telemetry_log
import ui_tview_main_window


LEFT_LEGEND_LOC=3
RIGHT_LEGEND_LOC=2


# TODO jpieper: Factor these out of tplot.py
def _get_data(value, name):
    fields = name.split('.')
    for field in fields:
        if isinstance(value, list):
            value = value[int(field)]
        else:
            value = getattr(value, field)
    return value


def _set_tree_widget_data(item, struct,
                          getter=lambda x, y: getattr(x, y),
                          required_size=0):
    if item.childCount() < required_size:
        for i in range(item.childCount(), required_size):
            subitem = QtGui.QTreeWidgetItem(item)
            subitem.setText(0, str(i))
    for i in range(item.childCount()):
        child = item.child(i)
        name = child.text(0)

        field = getter(struct, name)
        if isinstance(field, tuple) and child.childCount() > 0:
            _set_tree_widget_data(child, field)
        elif isinstance(field, list):
            _set_tree_widget_data(child, field,
                                  getter=lambda x, y: x[int(y)],
                                  required_size=len(field))
        else:
            child.setText(1, repr(field))


class RecordSignal(object):
    def __init__(self):
        self._index = 0
        self._callbacks = {}

    def connect(self, handler):
        result = self._index
        self._index += 1
        self._callbacks[result] = handler

        class Connection(object):
            def __init__(self, parent, index):
                self.parent = parent
                self.index = index

            def remove(self):
                del self.parent._callbacks[self.index]

        return Connection(self, result)

    def update(self, value):
        for handler in self._callbacks.itervalues():
            handler(value)


class PlotItem(object):
    def __init__(self, axis, plot_widget, name, signal):
        self.axis = axis
        self.plot_widget = plot_widget
        self.name = name
        self.line = None
        self.xdata = []
        self.ydata = []
        self.connection = signal.connect(self._handle_update)

    def _make_line(self):
        line = matplotlib.lines.Line2D([], [])
        line.set_label(self.name)
        line.set_color(self.plot_widget.COLORS[self.plot_widget.next_color])
        self.plot_widget.next_color = (
            self.plot_widget.next_color + 1) % len(self.plot_widget.COLORS)

        self.axis.add_line(line)
        self.axis.legend(loc=self.axis.legend_loc)

        self.line = line

    def remove(self):
        self.line.remove()
        self.connection.remove()
        # NOTE jpieper: matplotlib gives us no better way to remove a
        # legend.
        if len(self.axis.lines) == 0:
            self.axis.legend_ = None
        else:
            self.axis.legend(loc=self.axis.legend_loc)
        self.plot_widget.canvas.draw()

    def _handle_update(self, value):
        if self.plot_widget.paused:
            return

        if self.line is None:
            self._make_line()

        now = time.time()
        self.xdata.append(now)
        self.ydata.append(value)

        # Remove elements from the beginning until there is at most
        # one before the window.
        oldest_time = now - self.plot_widget.history_s
        oldest_index = None
        for i in range(len(self.xdata)):
            if self.xdata[i] >= oldest_time:
                oldest_index = i - 1
                break

        if oldest_index and oldest_index > 1:
            self.xdata = self.xdata[oldest_index:]
            self.ydata = self.ydata[oldest_index:]

        self.line.set_data(self.xdata, self.ydata)

        self.axis.relim()
        self.axis.autoscale()


class PlotWidget(QtGui.QWidget):
    COLORS = 'rbgcmyk'

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)

        self.history_s = 20.0
        self.next_color = 0
        self.paused = False

        self.figure = matplotlib.figure.Figure()
        self.canvas = FigureCanvas(self.figure)

        self.canvas.mpl_connect('key_press_event', self.handle_key_press)
        self.canvas.mpl_connect('key_release_event', self.handle_key_release)

        self.left_axis = self.figure.add_subplot(111)
        self.left_axis.grid()
        self.left_axis.fmt_xdata = lambda x: '%.3f' % x

        self.left_axis.legend_loc = LEFT_LEGEND_LOC

        self.right_axis = None

        def draw():
            # NOTE jpieper: For some reason, on the first repaint
            # event, the height is negative, which throws spurious
            # errors.  Paper over that here.
            l, b, w, h = self.figure.bbox.bounds
            if h < 0:
                return
            FigureCanvas.draw(self.canvas)
            self.canvas.repaint()

        self.canvas.draw = draw

        self.toolbar = backend_qt4agg.NavigationToolbar2QT(self.canvas, self)
        self.pause_action = QtGui.QAction(u'Pause', self)
        self.pause_action.setCheckable(True)
        self.pause_action.toggled.connect(self._handle_pause)
        self.toolbar.addAction(self.pause_action)

        layout = QtGui.QVBoxLayout(self)
        layout.addWidget(self.toolbar, 0)
        layout.addWidget(self.canvas, 1)

        self.canvas.setFocusPolicy(QtCore.Qt.ClickFocus)

    def _handle_pause(self, value):
        self.paused = value

    def add_plot(self, name, signal, axis_number):
        axis = self.left_axis
        if axis_number == 1:
            if self.right_axis is None:
                self.right_axis = self.left_axis.twinx()
                self.right_axis.legend_loc = RIGHT_LEGEND_LOC
            axis = self.right_axis
        item = PlotItem(axis, self, name, signal)
        return item

    def remove_plot(self, item):
        item.remove()

    def _get_axes_keys(self):
        result = []
        result.append(('1', self.left_axis))
        if self.right_axis:
            result.append(('2', self.right_axis))
        return result

    def handle_key_press(self, event):
        if event.key not in ['1', '2']:
            return
        for key, axis in self._get_axes_keys():
            if key == event.key:
                axis.set_navigate(True)
            else:
                axis.set_navigate(False)

    def handle_key_release(self, event):
        if event.key not in ['1', '2']:
            return
        for key, axis in self._get_axes_keys():
            axis.set_navigate(True)


class SizedTreeWidget(QtGui.QTreeWidget):
    def __init__(self, parent=None):
        QtGui.QTreeWidget.__init__(self, parent)
        self.setColumnCount(2)
        self.headerItem().setText(0, 'Name')
        self.headerItem().setText(1, 'Value')

    def sizeHint(self):
        return QtCore.QSize(350, 500)


class TviewConsoleWidget(HistoryConsoleWidget):
    line_input = QtCore.Signal(str)

    def __init__(self, *args, **kw):
        super(TviewConsoleWidget, self).__init__(*args, **kw)

        self._prompt = '>>> '
        self.clear()

    def sizeHint(self):
        return QtCore.QSize(600, 200)

    def add_text(self, data):
        assert data.endswith('\n') or data.endswith('\r')
        self._append_plain_text(data, before_prompt=True)
        self._control.moveCursor(QtGui.QTextCursor.End)

    def _handle_timeout(self):
        self._append_plain_text('%s\r\n' % time.time(),
                                before_prompt=True)
        self._control.moveCursor(QtGui.QTextCursor.End)

    def _is_complete(self, source, interactive):
        return True

    def _execute(self, source, hidden):
        self.line_input.emit(source)
        self._show_prompt(self._prompt)
        return True


class Record(object):
    def __init__(self, archive, tree_item):
        self.archive = archive
        self.tree_item = tree_item
        self.signals = {}

    def get_signal(self, name):
        if name not in self.signals:
            self.signals[name] = RecordSignal()

        return self.signals[name]

    def update(self, struct):
        for key, signal in self.signals.iteritems():
            value = _get_data(struct, key)
            signal.update(value)


class NoEditDelegate(QtGui.QStyledItemDelegate):
    def __init__(self, parent=None):
        QtGui.QStyledItemDelegate.__init__(self, parent=parent)

    def createEditor(self, parent, option, index):
        return None


def _get_item_name(item):
    name = item.text(0)
    while item.parent():
        name = item.parent().text(0) + '.' + name
        item = item.parent()

    return name


def _get_item_root(item):
    while item.parent():
        item = item.parent()
    return item.text(0)


class TviewMainWindow(QtGui.QMainWindow):
    STATE_LINE = 0
    STATE_CONFIG = 1
    STATE_TELEMETRY = 2
    STATE_SCHEMA = 3
    STATE_DATA = 4

    def __init__(self, options, parent=None):
        QtGui.QMainWindow.__init__(self, parent)

        self.options = options
        self.port = None
        self.default_rate = 100

        self._buffer = ''
        self._serial_timer = QtCore.QTimer()
        self._serial_timer.timeout.connect(self._poll_serial)
        self._serial_timer.start(10)

        self._serial_state = self.STATE_LINE

        self._telemetry_records = {}
        self._schema_name = None
        self._config_tree_items = {}

        self.ui = ui_tview_main_window.Ui_TviewMainWindow()
        self.ui.setupUi(self)

        self.ui.configTreeWidget = SizedTreeWidget()
        self.ui.configDock.setWidget(self.ui.configTreeWidget)

        self.ui.telemetryTreeWidget = SizedTreeWidget()
        self.ui.telemetryDock.setWidget(self.ui.telemetryTreeWidget)

        self.ui.telemetryTreeWidget.itemExpanded.connect(
            self._handle_tree_expanded)
        self.ui.telemetryTreeWidget.itemCollapsed.connect(
            self._handle_tree_collapsed)
        self.ui.telemetryTreeWidget.setContextMenuPolicy(
            QtCore.Qt.CustomContextMenu)
        self.ui.telemetryTreeWidget.customContextMenuRequested.connect(
            self._handle_telemetry_context_menu)

        self.ui.configTreeWidget.setItemDelegateForColumn(
            0, NoEditDelegate(self))
        self.ui.configTreeWidget.itemExpanded.connect(
            self._handle_config_expanded)
        self.ui.configTreeWidget.itemChanged.connect(
            self._handle_config_item_changed)

        self.ui.plotItemRemoveButton.clicked.connect(
            self._handle_plot_item_remove)

        self.console = TviewConsoleWidget()
        self.console.ansi_codes = False
        self.console.line_input.connect(self._handle_user_input)
        self.ui.consoleDock.setWidget(self.console)

        self.tabifyDockWidget(self.ui.configDock, self.ui.telemetryDock)

        layout = QtGui.QVBoxLayout(self.ui.plotHolderWidget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self.ui.plotHolderWidget.setLayout(layout)
        self.ui.plotWidget = PlotWidget(self.ui.plotHolderWidget)
        layout.addWidget(self.ui.plotWidget)

        def update_plotwidget(value):
            self.ui.plotWidget.history_s = value
        self.ui.historySpin.valueChanged.connect(update_plotwidget)

        self._config_callback = None

        QtCore.QTimer.singleShot(0, self._handle_startup)

    def _open(self):
        self.port = serial.Serial(
            port=self.options.serial,
            baudrate=self.options.baudrate,
            timeout=0.0)

        # Stop the spew.
        self.port.write('\r\n')
        self.port.write('tel stop\r\n')

        # Wait a short while, then eat everything we can.
        time.sleep(0.1)
        self.port.read(8192)

    def _handle_startup(self):
        self.console._control.setFocus()

    def _setup_device(self, callback):
        # When we start, get a listing of all configuration options
        # and all available telemetry channels.
        def after_config():
            self.update_telemetry(callback)
        self.update_config(after_config)

    def _poll_serial(self):
        if self.port is None:
            if os.path.exists(self.options.serial):
                self._open()
                self._setup_device(None)
            else:
                return

        try:
            data = self.port.read(8192)
        except serial.serialutil.SerialException:
            # We must have disconnected, close the port and try to
            # re-open.
            self.port.close()
            self.port = None
            return

        self._buffer += data

        while True:
            old_len = len(self._buffer)
            self._handle_serial_data()
            if len(self._buffer) == old_len:
                break

    def _handle_user_input(self, line):
        if self.port:
            self.port.write(line + '\n')

    def _handle_serial_data(self):
        if self._serial_state == self.STATE_LINE:
            self._handle_serial_line()
        elif self._serial_state == self.STATE_CONFIG:
            self._handle_config()
        elif self._serial_state == self.STATE_TELEMETRY:
            self._handle_telemetry()
        elif self._serial_state == self.STATE_SCHEMA:
            self._handle_schema()
        elif self._serial_state == self.STATE_DATA:
            self._handle_data()
        else:
            assert False

    def _handle_serial_line(self):
        line = self._get_serial_line()
        if line is None:
            return

        display = True
        if line == '':
            display = False

        if line.startswith('schema '):
            self._serial_state = self.STATE_SCHEMA
            self._schema_name = line.split(' ', 1)[1].strip()
        elif line.startswith('emit '):
            self._serial_state = self.STATE_DATA
            self._schema_name = line.split(' ', 1)[1].strip()
            display = False

        if display:
            self.console.add_text(line + '\n')

    def _get_serial_line(self):
        delim = '[\r\n]'
        if not re.search(delim, self._buffer):
            return

        line, rest = re.split('[\n\r]', self._buffer, maxsplit=1)
        self._buffer = rest
        return line

    def update_config(self, callback):
        # Clear out our config tree.
        self.ui.configTreeWidget.clear()
        self._config_tree_items = {}

        self._config_callback = callback
        self.write_line('conf enumerate\r\n')

        # TODO jpieper: In the current protocol this is racy, as there
        # is no header on the config enumeration.  I should probably
        # add one.
        self._serial_state = self.STATE_CONFIG

    def _handle_config(self):
        line = self._get_serial_line()
        if not line:
            return

        self.console.add_text(line + '\n')

        if line.startswith('OK'):
            # We're done with config now.
            self._serial_state = self.STATE_LINE
            cbk, self._config_callback = self._config_callback, None
            if cbk:
                cbk()
        else:
            # Add it into our tree view.
            key, value = line.split(' ', 1)
            name, rest = key.split('.', 1)
            if name not in self._config_tree_items:
                item = QtGui.QTreeWidgetItem()
                item.setText(0, name)
                self.ui.configTreeWidget.addTopLevelItem(item)
                self._config_tree_items[name] = item

            def add_config(item, key, value):
                if key == '':
                    item.setText(1, value)
                    item.setFlags(QtCore.Qt.ItemIsEditable |
                                  QtCore.Qt.ItemIsSelectable |
                                  QtCore.Qt.ItemIsEnabled)
                    return

                fields = key.split('.', 1)
                this_field = fields[0]
                next_key = ''
                if len(fields) > 1:
                    next_key = fields[1]

                child = None
                # See if we already have an appropriate child.
                for i in range(item.childCount()):
                    if item.child(i).text(0) == this_field:
                        child = item.child(i)
                        break
                if child is None:
                    child = QtGui.QTreeWidgetItem(item)
                    child.setText(0, this_field)
                add_config(child, next_key, value)

            add_config(self._config_tree_items[name], rest, value)

            self.ui.configTreeWidget.resizeColumnToContents(0)

    def update_telemetry(self, callback):
        self.ui.telemetryTreeWidget.clear()
        self._telemetry_records = {}

        self._telemetry_callback = callback
        self.write_line('tel list\r\n')

        self._serial_state = self.STATE_TELEMETRY

    def write_line(self, line):
        self.console.add_text(line)
        if self.port:
            self.port.write(line)

    def _handle_telemetry(self):
        line = self._get_serial_line()
        if not line:
            return

        self.console.add_text(line + '\n')

        if line.startswith('OK'):
            # Now we need to start getting schemas.
            self._serial_state = self.STATE_LINE
            self._update_schema()
        else:
            name = line.strip()
            self._telemetry_records[name] = None

    def _update_schema(self):
        # Find a channel we don't have a schema for and request it.
        for name in self._telemetry_records.keys():
            if self._telemetry_records[name] is None:
                self.write_line('tel schema %s\r\n' % name)
                self._serial_state = self.STATE_LINE
                return

        self._serial_state = self.STATE_LINE
        # Guess we are done.  Update our tree view.
        self.ui.telemetryTreeWidget.resizeColumnToContents(0)
        cbk, self._telemetry_callback = self._telemetry_callback, None
        if cbk:
            cbk()

    def _handle_schema(self):
        schema = self._handle_sized_block()
        if not schema:
            return

        name, self._schema_name = self._schema_name, None

        if name in self._telemetry_records:
            if self._telemetry_records[name]:
                return

        archive = telemetry_archive.ReadArchive(schema, name)
        item = self._add_schema_to_tree(name, archive)

        self._telemetry_records[name] = Record(archive, item)

        self.console.add_text('<schema name=%s>\n' % name)

        # Now look to see if there are any more we should request.
        self._update_schema()

    def _handle_data(self):
        data = self._handle_sized_block()
        if not data:
            return

        name, self._schema_name = self._schema_name, None

        if name not in self._telemetry_records:
            return

        record = self._telemetry_records[name]
        struct = record.archive.deserialize(data)
        _set_tree_widget_data(record.tree_item, struct)
        record.update(struct)
        self.ui.plotWidget.canvas.draw()

        self._serial_state = self.STATE_LINE

    def _handle_sized_block(self):
        # Wait until we have the complete schema in the buffer.  It
        # will start with the final newline from the first line.
        if len(self._buffer) < 5:
            return

        size = struct.unpack('<I', self._buffer[1:5])[0]
        if size > 2 ** 24:
            # Whoops, probably bogus.
            print 'Invalid schema size, skipping whatever we were doing.'
            self._serial_state = self.STATE_LINE
            return

        if len(self._buffer) < 5 + size:
            return

        block = self._buffer[5:5+size]
        self._buffer = self._buffer[5+size:]
        return block

    def _add_schema_to_tree(self, name, archive):
        item = QtGui.QTreeWidgetItem()
        item.setText(0, name)
        self.ui.telemetryTreeWidget.addTopLevelItem(item)

        # TODO jpieper: Factor this out of tplot.py.
        def add_item(parent, element):
            if 'fields' not in element:
                return
            for field in element['fields']:
                name = field['name']

                item = QtGui.QTreeWidgetItem(parent)
                item.setText(0, name)

                if 'children' in field:
                    for child in field['children']:
                        add_item(item, child)

        add_item(item, archive.root)
        return item

    def _handle_tree_expanded(self, item):
        self.ui.telemetryTreeWidget.resizeColumnToContents(0)
        if item.parent() is None:
            # OK, since we're a top level node, request to start
            # receiving updates.
            name = item.text(0)
            self.write_line('tel fmt %s 0\r\n' % name)
            self.write_line('tel rate %s %d\r\n' % (name, self.default_rate))

    def _handle_tree_collapsed(self, item):
        if item.parent() is None:
            # Stop this guy.
            name = item.text(0)
            self.write_line('tel rate %s 0\r\n' % name)

    def _handle_telemetry_context_menu(self, pos):
        item = self.ui.telemetryTreeWidget.itemAt(pos)
        if item.childCount() > 0:
            return

        menu = QtGui.QMenu(self)
        left_action = menu.addAction('Plot Left')
        right_action = menu.addAction('Plot Right')
        menu.addSeparator()
        copy_name = menu.addAction('Copy Name')
        copy_value = menu.addAction('Copy Value')

        requested = menu.exec_(self.ui.telemetryTreeWidget.mapToGlobal(pos))

        if requested == left_action or requested == right_action:
            name = _get_item_name(item)
            root = _get_item_root(item)

            record = self._telemetry_records[root]

            leaf = name.split('.', 1)[1]
            axis = 0
            if requested == right_action:
                axis = 1
            plot_item = self.ui.plotWidget.add_plot(
                name, record.get_signal(leaf), axis)
            self.ui.plotItemCombo.addItem(name, plot_item)
        elif requested == copy_name:
            QtGui.QApplication.clipboard().setText(item.text(0))
        elif requested == copy_value:
            QtGui.QApplication.clipboard().setText(item.text(1))
        else:
            # The user cancelled.
            pass

    def _handle_config_expanded(self, item):
        self.ui.configTreeWidget.resizeColumnToContents(0)

    def _handle_config_item_changed(self, item, column):
        if self._serial_state == self.STATE_CONFIG:
            return
        value = item.text(1)
        name = _get_item_name(item)

        self.write_line('conf set %s %s\r\n' % (name, value))

    def _handle_plot_item_remove(self):
        index = self.ui.plotItemCombo.currentIndex()

        if index < 0:
            return

        item = self.ui.plotItemCombo.itemData(index)
        self.ui.plotWidget.remove_plot(item)
        self.ui.plotItemCombo.removeItem(index)


def main():
    usage, description = __doc__.split('\n\n', 1)
    parser = optparse.OptionParser(usage=usage, description=description)

    parser.add_option('--serial', '-s', default='/dev/ttyACM0')
    parser.add_option('--baudrate', '-b', type='int', default=115200)

    options, args = parser.parse_args()
    assert len(args) == 0

    app = QtGui.QApplication(sys.argv)

    tv = TviewMainWindow(options)
    tv.show()

    app.exec_()


if __name__ == '__main__':
    main()
