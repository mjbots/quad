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
import sys
import time

from IPython.external.qt import QtCore, QtGui
from IPython.qt.console.history_console_widget import HistoryConsoleWidget

SCRIPT_PATH=os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(SCRIPT_PATH, '../python'))
sys.path.append(os.path.join(SCRIPT_PATH, 'build-x86_64'))

import telemetry_archive
import telemetry_log
import ui_tview_main_window

class TviewConsoleWidget(HistoryConsoleWidget):
    line_input = QtCore.Signal(str)

    def __init__(self, *args, **kw):
        super(TviewConsoleWidget, self).__init__(*args, **kw)

        self._prompt = '>>> '
        self.clear()

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


# TODO jpieper: Factor this out of tplot.py
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


class TviewMainWindow(QtGui.QMainWindow):
    STATE_LINE = 0
    STATE_CONFIG = 1
    STATE_TELEMETRY = 2
    STATE_SCHEMA = 3
    STATE_DATA = 4

    def __init__(self, port, parent=None):
        QtGui.QMainWindow.__init__(self, parent)

        self.port = port

        self._buffer = ''
        self._serial_timer = QtCore.QTimer()
        self._serial_timer.timeout.connect(self._poll_serial)
        self._serial_timer.start(10)

        self._serial_state = self.STATE_LINE

        self._telemetry_records = {}
        self._schema_name = None

        self.ui = ui_tview_main_window.Ui_TviewMainWindow()
        self.ui.setupUi(self)

        self.console = TviewConsoleWidget()
        self.console.ansi_codes = False
        self.console.line_input.connect(self._handle_user_input)
        self.ui.consoleDock.setWidget(self.console)

        self._config_callback = None

        QtCore.QTimer.singleShot(0, self._handle_startup)

    def _handle_startup(self):
        self.console._control.setFocus()

        # When we start, get a listing of all configuration options
        # and all available telemetry channels.
        def after_config():
            self.update_telemetry(None)
        self.update_config(after_config)

    def _poll_serial(self):
        data = self.port.read(8192)
        self._buffer += data

        if len(data):
            self._handle_serial_data()

    def _handle_user_input(self, line):
        self.port.write(line + '\n')

    def _handle_serial_data(self):
        if self._serial_state == self.STATE_LINE:
            self._handle_serial_line()
        elif self._serial_state == self.STATE_CONFIG:
            self._handle_config()
        elif self._serial_state == self.STATE_TELEMETRY:
            self._handle_telemetry()
        elif self._serial_schema == self.STATE_SCHEMA:
            self._handle_schema()
        elif self._serial_schema == self.STATE_DATA:
            self._handle_data()
        else:
            assert False

    def _handle_serial_line(self):
        line = self._get_serial_line()
        if line is None:
            return

        self.console.add_text(line + '\n')

        if line.startswith('schema '):
            self._serial_state = self.STATE_SCHEMA
            self._schema_name = line.split(' ', 1)[1].strip()
        elif line.startswith('emit '):
            self._serial_state = self.STATE_DATA
            self._schema_name = line.split(' ', 1)[1].strip()

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
            print "got config:", line

    def update_telemetry(self, callback):
        self.ui.telemetryTreeWidget.clear()
        self._telemetry_records = {}

        self._telemetry_callback = callback
        self.write_line('tel list\r\n')

        self._serial_state = self.STATE_TELEMETRY

    def write_line(self, line):
        self.console.add_text(line)
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
            print 'got telemetry:', line

    def _update_schema(self):
        # Find a channel we don't have a schema for and request it.
        for name in self._telemetry_records.keys():
            if self._telemetry_records[name] is None:
                self.write_line('tel schema %s\r\n' % name)
                return

        # Guess we are done.  Update our tree view.
        self._update_schema_tree()
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

        archive = telemetry_archive.ReadArchivePython(schema)
        item = self._add_schema_to_tree(name, archive)

        self._telemetry_schemas[name] = Record(archive, item)

        self.add_text('<schema name=%s>\n' % name)

        # Now look to see if there are any more we should request.
        self._update_schema()

    def _handle_data(self):
        data = self._handle_sized_block()
        if not data:
            return

        name, self._schema_name = self._schema_name, None

        if name not in self._telemetry_schemas:
            return

        record = self._telemetry_schemas[name]
        struct = record.archive.deserialize(data)
        _set_tree_widget_data(record.item, struct)

    def _handle_sized_block(self):
        # Wait until we have the complete schema in the buffer.  It
        # will start with the final newline from the first line.
        if len(self._buffer) < 5:
            return

        size = struct.unpack('<I', buffer[1:5])[0]
        if size > 2 ** 24:
            # Whoops, probably bogus.
            print 'Invalid schema size, skipping whatever we were doing.'
            self._serial_state = STATE_LINE
            return

        if len(self._buffer) < 5 + size:
            return

        block = self._buffer[5:5+size]
        self._buffer = self._buffer[5+size:]
        return block

    def _add_schema_to_tree(self, name, archive):
        item = QtGui.QTreeWidgetItem()
        item.setText(name)
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


def main():
    usage, description = __doc__.split('\n\n', 1)
    parser = optparse.OptionParser(usage=usage, description=description)

    parser.add_option('--serial', '-s', default='/dev/ttyACM0')
    parser.add_option('--baudrate', '-b', type='int', default=115200)

    options, args = parser.parse_args()
    assert len(args) == 0

    port = serial.Serial(port=options.serial, baudrate=options.baudrate,
                         timeout=0.0)
    # Stop the spew.
    port.write('\r\n')
    #port.write('tel stop\r\n')

    app = QtGui.QApplication(sys.argv)

    tv = TviewMainWindow(port)
    tv.show()

    app.exec_()


if __name__ == '__main__':
    main()
