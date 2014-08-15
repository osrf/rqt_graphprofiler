import sys

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

import rospy
import rosprofiler_adapter
from diarc import *
from diarc import topology
from diarc import qt_view
class VisualizerPlugin(Plugin):
    def __init__(self, context):
        super(VisualizerPlugin, self).__init__(context)
        self.setObjectName('VisualizerPlugin')
        
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                        dest="quiet", help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())

        # Create QWidget
#         self._widget = QWidget()
#         context.add_widget(self._widget)
        self._view = qt_view.QtView()
        self._adapter = rosprofiler_adapter.ROSProfileAdapter(self._view)
        context.add_widget(self._view)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
