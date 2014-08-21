#!/usr/bin/env python
# Usage:
# ./run.py topology_plot data/v5.xml

import sys
sys.dont_write_bytecode = True
import inspect
import logging

def asciiview(args):
    from diarc import parser
    from ascii_view import ascii_view
    from diarc import base_adapter
    topology = parser.parseFile(args[0])
    view = ascii_view.AsciiView()
    adapter = base_adapter.BaseAdapter(topology, view)
    adapter._update_view()

def qtview(args):
    try:
        import python_qt_binding.QtGui
    except:
        print "Error: python_qt_binding not installed."
        print "Please install using `sudo pip install python_qt_binding`"
        exit(-1)
    from diarc import parser
    from qt_view import qt_view
    from diarc import base_adapter
    topology = parser.parseFile(args[0])
    app = python_qt_binding.QtGui.QApplication(sys.argv)
    view = qt_view.QtView()
    adapter = base_adapter.BaseAdapter(topology, view)
    adapter._update_view()
    view.activateWindow()
    view.raise_()
    sys.exit(app.exec_())

def rosview():
    try:
        import python_qt_binding.QtGui
    except:
        print "Error: python_qt_binding not installed."
        print "Please install using `sudo pip install python_qt_binding`"
        exit(-1)
    import qt_view
    import ros.ros_adapter
    app = python_qt_binding.QtGui.QApplication([])
    view = qt_view.QtView()
    adapter = ros.ros_adapter.RosAdapter(view)
    adapter.update_model()
    view.activateWindow()
    view.raise_()
    sys.exit(app.exec_())




if __name__=="__main__":
    available_tests = dict(inspect.getmembers(sys.modules[__name__],inspect.isfunction))
    
    # Enable the next to rows to perform profiling
#     rosview()
#     exit()
    logging.basicConfig(level=logging.DEBUG)
    log = logging.getLogger('main')

    if len(sys.argv) < 2 or sys.argv[1] not in available_tests:
        print "Usage:\n ./test.py <test> [parameters]\n"
        print "Tests available:",available_tests.keys()
        exit(0)
    elif len(sys.argv)>2:
        available_tests[sys.argv[1]](sys.argv[2:])
    else:
        available_tests[sys.argv[1]]()

