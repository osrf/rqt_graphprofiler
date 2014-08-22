# Copyright 2014 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

""" Blacklist
Provides a blacklist editing dialog.

from python_qt_binding.QtGui import QApplication
from blacklist import BlacklistDialog

app = QApplication(sys.argv)
current_blacklist = ['hare', 'hyrax', 'camel', 'pig']
revised_blacklist = BlacklistDialog.get_blacklist(values=current_blacklist)
"""

from python_qt_binding.QtGui import QDialog
from python_qt_binding.QtGui import QHBoxLayout
from python_qt_binding.QtGui import QListView
from python_qt_binding.QtGui import QPushButton
from python_qt_binding.QtGui import QStringListModel
from python_qt_binding.QtGui import QVBoxLayout
from python_qt_binding.QtGui import QIcon


class BlacklistDialog(QDialog):
    """ BlacklistDialog """
    def __init__(self, parent=None, current_values=None):
        super(BlacklistDialog, self).__init__(parent)
        self.setWindowTitle("Blacklist")
        vbox = QVBoxLayout()
        self.setLayout(vbox)

        self._blacklist = Blacklist()
        if isinstance(current_values, list):
            for val in current_values:
                self._blacklist.append(val)
        vbox.addWidget(self._blacklist)

        controls_layout = QHBoxLayout()
        add_button = QPushButton(icon=QIcon.fromTheme('list-add'))
        rem_button = QPushButton(icon=QIcon.fromTheme('list-remove'))
        ok_button = QPushButton("Ok")
        cancel_button = QPushButton("Cancel")
        add_button.clicked.connect(self._add_item)
        rem_button.clicked.connect(self._remove_item)
        ok_button.clicked.connect(self.accept)
        cancel_button.clicked.connect(self.reject)

        controls_layout.addWidget(add_button)
        controls_layout.addWidget(rem_button)
        controls_layout.addStretch(0)
        controls_layout.addWidget(ok_button)
        controls_layout.addWidget(cancel_button)
        vbox.addLayout(controls_layout)

    def _add_item(self):
        """ Adds a new default item to the list """
        self._blacklist.append("new item")

    def _remove_item(self):
        """ Removes any selected items from the list """
        self._blacklist.remove_selected()

    def get_values(self):
        """ returns the most recent values of the blacklist """
        return self._blacklist.get_values()

    @staticmethod
    def get_blacklist(parent=None, values=None):
        """ Get a modified blacklist
        :param list values: current blacklisted values to populate the dialog with
        :returns: revised list if Ok is pressed, else values
        """
        dialog = BlacklistDialog(parent=parent, current_values=values)
        result = dialog.exec_()
        return dialog.get_values() if result == QDialog.Accepted else values


class Blacklist(QListView):
    """ an editable list of strings """
    def __init__(self, parent=None):
        super(Blacklist, self).__init__(parent)
        self._list = list()
        self._model = QStringListModel()
        self._model.setStringList(self._list)
        self.setModel(self._model)
        self.setDragEnabled(True)

        # Keep track of the selected value
        self._selected_index = None
        self.clicked.connect(self._selected)

        self._model.dataChanged.connect(self._datachanged)

    def _selected(self, index):
        """ registers when an item is selected """
        self._selected_index = index.row()

    def _datachanged(self, tl, br):
        """ Reorder list alphabetically when things change """
        self._list = self._model.stringList()
        self._list.sort()
        self._model.setStringList(self._list)

    def append(self, val):
        """ Adds a value to the list """
        self._list = self._model.stringList()
        self._list.append(val)
        self._model.setStringList(self._list)
        self._selected_index = None

    def remove_selected(self):
        """ removes any selected value from the list """
        if self._selected_index is not None:
            self._list = self._model.stringList()
            self._list.pop(self._selected_index)
            self._model.setStringList(self._list)
            self._selected_index = None

    def get_values(self):
        """ returns the values stored in the list """
        return [str(val) for val in self._model.stringList()]
