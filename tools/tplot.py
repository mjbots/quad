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

import datetime
import os
import sys
import time

import matplotlib

matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4'] = 'PySide'

from matplotlib.backends import backend_qt4agg
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

SCRIPT_PATH=os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(SCRIPT_PATH, '../python'))
sys.path.append(os.path.join(SCRIPT_PATH, 'build'))
import ui_tplot_main_window

import telemetry_log

AXES = ['Left', 'Right', '3', '4']

LEGEND_LOC = {
    'Left': 2,
    'Right': 1,
    '3': 7,
    '4': 4
    }


class BoolGuard(object):
    def __init__(self):
        self.value = False

    def __enter__(self):
        self.value = True

    def __exit__(self, type, value, traceback):
        self.value = False

    def active(self):
        return self.value


class Log(object):
    def __init__(self, filename):
        self.reader = telemetry_log.BulkReader(open(filename))
        self.records = self.reader.records()
        self.all = None

    def get_all(self):
        if self.all:
            return
        self.all = self.reader.get()


def _bisect(array, item, key):
    if len(array) == 0:
        return None
    if item < key(array[0]):
        return None

    lower = 0
    upper = len(array)

    while abs(lower - upper) > 1:
        mid = (lower + upper) / 2
        value = key(array[mid])
        if item < value:
            upper = mid
        else:
            lower = mid

    return lower


def _clear_tree_widget(item):
    item.setText(1, '')
    for i in range(item.childCount()):
        child = item.child(i)
        _clear_tree_widget(child)


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
            child.setText(1, str(field))


def _get_data(value, name):
    fields = name.split('.')
    for field in fields:
        if isinstance(value, list):
            value = value[int(field)]
        else:
            value = getattr(value, field)
    return value


class Tplot(QtGui.QMainWindow):
    def __init__(self):
        super(Tplot, self).__init__()

        self.ui = ui_tplot_main_window.Ui_TplotMainWindow()
        self.ui.setupUi(self)

        self.figure = matplotlib.figure.Figure()
        self.canvas = FigureCanvas(self.figure)

        self.canvas.mpl_connect('motion_notify_event', self.handle_mouse)
        self.canvas.mpl_connect('key_press_event', self.handle_key_press)
        self.canvas.mpl_connect('key_release_event', self.handle_key_release)

        # Make QT drawing not be super slow.  See:
        # https://github.com/matplotlib/matplotlib/issues/2559/
        def draw():
            FigureCanvas.draw(self.canvas)
            self.canvas.repaint()

        self.canvas.draw = draw

        self.left_axis = self.figure.add_subplot(111)
        self.left_axis.tplot_name = 'Left'

        self.axes = {
            'Left' : self.left_axis,
            }

        layout = QtGui.QVBoxLayout(self.ui.plotFrame)
        layout.addWidget(self.canvas, 1)

        self.toolbar = backend_qt4agg.NavigationToolbar2QT(self.canvas, self)
        self.addToolBar(self.toolbar)

        self.canvas.setFocusPolicy(QtCore.Qt.ClickFocus)
        self.canvas.setFocus()

        self.log = None
        self.COLORS = 'rgbcmyk'
        self.next_color = 0

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.handle_timeout)

        self.time_start = None
        self.time_end = None
        self.time_current = None

        self.ui.recordCombo.currentIndexChanged.connect(
            self.handle_record_combo)
        self.ui.addPlotButton.clicked.connect(self.handle_add_plot_button)
        self.ui.removeButton.clicked.connect(self.handle_remove_button)
        self.ui.treeWidget.itemExpanded.connect(self.handle_item_expanded)
        self.tree_items = []
        self.ui.treeWidget.header().setResizeMode(
            QtGui.QHeaderView.ResizeToContents)
        self.ui.timeSlider.valueChanged.connect(self.handle_time_slider)
        self._updating_slider = BoolGuard()

        self.ui.fastReverseButton.clicked.connect(
            self.handle_fast_reverse_button)
        self.ui.stepBackButton.clicked.connect(
            self.handle_step_back_button)
        self.ui.playReverseButton.clicked.connect(
            self.handle_play_reverse_button)
        self.ui.stopButton.clicked.connect(self.handle_stop_button)
        self.ui.playButton.clicked.connect(self.handle_play_button)
        self.ui.stepForwardButton.clicked.connect(
            self.handle_step_forward_button)
        self.ui.fastForwardButton.clicked.connect(
            self.handle_fast_forward_button)

    def open(self, filename):
        try:
            maybe_log = Log(filename)
        except Exception as e:
            QtGui.QMessageBox.warning(self, 'Could not open log',
                                      'Error: ' + str(e))
            raise
            return

        # OK, we're good, clear out our UI.
        self.ui.treeWidget.clear()
        self.tree_items = []
        self.ui.recordCombo.clear()
        self.ui.xCombo.clear()
        self.ui.yCombo.clear()

        self.log = maybe_log
        for name in self.log.records.keys():
            self.ui.recordCombo.addItem(name)

            item = QtGui.QTreeWidgetItem()
            item.setText(0, name)
            self.ui.treeWidget.addTopLevelItem(item)
            self.tree_items.append(item)

            exemplar = self.log.records[name]
            def add_item(parent, element):
                if 'fields' not in element:
                    return
                for field in element['fields']:
                    name = field['name']

                    item = QtGui.QTreeWidgetItem(parent)
                    item.setText(0, name)

                    if 'nelements' in field:
                        child = field['children'][0]
                        # If this is a type with only one child, just
                        # fall down to it.
                        if 'children' in child and len(child['children'] == 1):
                            child = ['children'][0]
                        for i in range(field['nelements']):
                            subitem = QtGui.QTreeWidgetItem(item)
                            subitem.setText(0, str(i))
                            add_item(subitem, child)
                    elif 'children' in field:
                        for child in field['children']:
                            add_item(item, child)
            add_item(item, exemplar)

    def handle_record_combo(self):
        record = self.ui.recordCombo.currentText()
        self.ui.xCombo.clear()
        self.ui.yCombo.clear()

        exemplar = self.log.records[record]
        default_x = None
        index = [0, None]

        def add_item(index, parent, element):
            if 'fields' not in element:
                return
            for field in element['fields']:
                name = field['name']
                this_name = parent
                if len(this_name):
                    this_name += '.'
                this_name += name

                if 'nelements' in field:
                    child = field['children'][0]
                    if 'children' in child and len(child['children']) == 1:
                        child = ['children'][0]
                    for i in range(field['nelements']):
                        add_item(index, this_name + "." + str(i), child)
                elif 'children' in field:
                    for child in field['children']:
                        add_item(index, this_name, child)
                else:
                    self.ui.xCombo.addItem(this_name)
                    self.ui.yCombo.addItem(this_name)

                    if name == 'timestamp':
                        index[1] = index[0]

                    index[0] += 1

        add_item(index, '', exemplar)
        default_x = index[1]

        if default_x:
            self.ui.xCombo.setCurrentIndex(default_x)

    def handle_add_plot_button(self):
        self.log.get_all()

        record = self.ui.recordCombo.currentText()
        xname = self.ui.xCombo.currentText()
        yname = self.ui.yCombo.currentText()

        data = self.log.all[record]
        xdata = [_get_data(x, xname) for x in data]
        ydata = [_get_data(x, yname) for x in data]

        line = matplotlib.lines.Line2D(xdata, ydata)
        line.tplot_record_name = record
        if 'timestamp' in [x['name'] for x in self.log.records[record]['fields']]:
            line.tplot_has_timestamp = True
        line.tplot_xname = xname
        line.tplot_yname = yname
        label = self.make_label(record, xname, yname)
        line.set_label(label)
        line.set_color(self.COLORS[self.next_color])
        self.next_color = (self.next_color + 1) % len(self.COLORS)

        axis = self.get_current_axis()

        axis.add_line(line)
        axis.relim()
        axis.autoscale_view()
        axis.legend(loc=LEGEND_LOC[axis.tplot_name])

        self.ui.plotsCombo.addItem(label, line)
        self.ui.plotsCombo.setCurrentIndex(self.ui.plotsCombo.count() - 1)

        self.canvas.draw()

    def make_label(self, record, xname, yname):
        if xname == 'timestamp':
            return '%s.%s' % (record, yname)
        return '%s %s vs. %s' % (record, yname, xname)

    def get_current_axis(self):
        requested = self.ui.axisCombo.currentText()
        maybe_result = self.axes.get(requested, None)
        if maybe_result:
            return maybe_result

        result = self.left_axis.twinx()
        self.axes[requested] = result
        result.tplot_name = requested

        return result

    def get_all_axes(self):
        return self.axes.values()

    def handle_remove_button(self):
        index = self.ui.plotsCombo.currentIndex()
        if index < 0:
            return
        line = self.ui.plotsCombo.itemData(index)
        if hasattr(line, 'tplot_marker'):
            line.tplot_marker.remove()
        line.remove()
        self.ui.plotsCombo.removeItem(index)

        self.canvas.draw()

    def handle_item_expanded(self):
        self.update_timeline()

    def update_timeline(self):
        if self.time_start is not None:
            return

        self.log.get_all()

        # Look through all the records for those which have a
        # "timestamp" field.  Find the minimum and maximum of each.
        for record, exemplar in self.log.records.iteritems():
            if not 'timestamp' in [x['name'] for x in exemplar['fields']]:
                continue

            these_times = [x.timestamp for x in self.log.all[record]]
            if len(these_times) == 0:
                continue
            this_min = min(these_times)
            this_max = max(these_times)

            if self.time_start is None or this_min < self.time_start:
                self.time_start = this_min
            if self.time_end is None or this_max > self.time_end:
                self.time_end = this_max

        self.time_current = self.time_start
        self.update_time(self.time_current, update_slider=False)

    def handle_mouse(self, event):
        if not event.inaxes:
            return
        self.statusBar().showMessage('%f,%f' % (event.xdata, event.ydata))

    def handle_key_press(self, event):
        if event.key not in ['1', '2', '3', '4']:
            return
        index = ord(event.key) - ord('1')
        for key, value in self.axes.iteritems():
            if key == AXES[index]:
                value.set_navigate(True)
            else:
                value.set_navigate(False)

    def handle_key_release(self, event):
        if event.key not in ['1', '2', '3', '4']:
            return
        for key, value in self.axes.iteritems():
            value.set_navigate(True)

    def update_time(self, new_time, update_slider=True):
        new_time = max(self.time_start, min(self.time_end, new_time))
        self.time_current = new_time

        # Update the tree view.
        self.update_tree_view(new_time)

        # Update dots on the plot.
        self.update_plot_dots(new_time)

        # Update the text fields.
        dt = datetime.datetime.utcfromtimestamp(new_time)
        self.ui.clockEdit.setText('%04d-%02d-%02d %02d:%02d:%02.3f' % (
                dt.year, dt.month, dt.day,
                dt.hour, dt.minute, dt.second + dt.microsecond / 1e6))
        self.ui.elapsedEdit.setText('%.3f' % (new_time - self.time_start))

        if update_slider:
            with self._updating_slider:
                elapsed = new_time - self.time_start
                total_time = self.time_end - self.time_start
                self.ui.timeSlider.setValue(
                    int(1000 * elapsed / total_time))

    def handle_time_slider(self):
        if self._updating_slider.active():
            return

        if self.time_end is None or self.time_start is None:
            return

        total_time = self.time_end - self.time_start
        current = self.ui.timeSlider.value() / 1000.0
        self.update_time(self.time_start + current * total_time,
                         update_slider=False)

    def update_tree_view(self, time):
        for item in self.tree_items:
            name = item.text(0)
            all_data = self.log.all[name]

            this_data_index = _bisect(all_data, time,
                                      key=lambda x: x.timestamp)
            if this_data_index is None:
                _clear_tree_widget(item)
            else:
                this_data = all_data[this_data_index]
                _set_tree_widget_data(item, this_data)

    def update_plot_dots(self, new_time):
        updated = False
        for axis in self.get_all_axes():
            for line in axis.lines:
                if not hasattr(line, 'tplot_record_name'):
                    continue
                if not hasattr(line, 'tplot_has_timestamp'):
                    continue

                all_data = self.log.all[line.tplot_record_name]
                this_index = _bisect(all_data, new_time, lambda x: x.timestamp)
                if this_index is None:
                    continue

                this_data = all_data[this_index]

                if not hasattr(line, 'tplot_marker'):
                    line.tplot_marker = matplotlib.lines.Line2D([], [])
                    line.tplot_marker.set_marker('o')
                    line.tplot_marker.set_color(line._color)
                    self.left_axis.add_line(line.tplot_marker)

                updated = True
                xdata = [_get_data(this_data, line.tplot_xname)]
                ydata = [_get_data(this_data, line.tplot_yname)]
                line.tplot_marker.set_data(xdata, ydata)

        if updated:
            self.canvas.draw()


    def handle_fast_reverse_button(self):
        self.play_start(-self.ui.fastReverseSpin.value())

    def handle_step_back_button(self):
        self.play_stop()
        self.update_time(self.time_current - self.ui.stepBackSpin.value())

    def handle_play_reverse_button(self):
        self.play_start(-1.0)

    def handle_stop_button(self):
        self.play_stop()

    def handle_play_button(self):
        self.play_start(1.0)

    def handle_step_forward_button(self):
        self.play_stop()
        self.update_time(self.time_current + self.ui.stepForwardSpin.value())

    def handle_fast_forward_button(self):
        self.play_start(self.ui.fastForwardSpin.value())

    def play_stop(self):
        self.speed = None
        self.last_time = None
        self.timer.stop()

    def play_start(self, speed):
        self.speed = speed
        self.last_time = time.time()
        self.timer.start(100)

    def handle_timeout(self):
        assert self.last_time is not None
        this_time = time.time()
        delta_t = this_time - self.last_time
        self.last_time = this_time

        self.update_time(self.time_current + delta_t * self.speed)


def main():
    app = QtGui.QApplication(sys.argv)
    app.setApplicationName('tplot')

    tplot = Tplot()
    tplot.show()

    if len(sys.argv) > 0:
        tplot.open(sys.argv[1])

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
