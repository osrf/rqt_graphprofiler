#!/usr/bin/env python
import PyQt4.QtGui
from diarc import *
from diarc import topology
from diarc import qt_view
# import graphprofiler_adapter
import rosprofiler_adapter
import rospy
import sys
sys.dont_write_bytecode = True

if __name__ == '__main__':
    rospy.init_node('test')
    app = PyQt4.QtGui.QApplication([])
    view = qt_view.QtView()
    adapter = rosprofiler_adapter.ROSProfileAdapter(view)
    view.activateWindow()
    view.raise_()
    sys.exit(app.exec_())

