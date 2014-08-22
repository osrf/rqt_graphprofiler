cmake_minimum_required(VERSION 2.8.3)
project(rqt_graphprofiler)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
#   message_generation
  std_msgs
)

catkin_python_setup()

## These msg types are for republishing visualized data, not implemented yet
# add_message_files(DIRECTORY msg
#   FILES
#   TopicProfile.msg
#   NodeProfile.msg
#   HostProfile.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

catkin_package()

install(PROGRAMS
  scripts/rqt_graphprofiler
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# catkin_add_nosetests(test)
