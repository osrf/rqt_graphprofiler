#!/usr/bin/env python
import PyQt4.QtGui
from diarc import *
# import graphprofiler_adapter
import rosprofiler_adapter
import rospy
import sys
sys.dont_write_bytecode = True

if __name__ == '__main__':
    rospy.init_node('test')
    app = PyQt4.QtGui.QApplication([])
    view = qt_view.QtView()
#     adapter = graphprofiler_adapter.GraphProfileAdapter(view)
    adapter = rosprofiler_adapter.ROSProfileAdapter(view)
#     adapter.update_model()
    view.activateWindow()
    view.raise_()
    sys.exit(app.exec_())

