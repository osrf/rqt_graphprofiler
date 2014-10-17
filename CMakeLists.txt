cmake_minimum_required(VERSION 2.8.3)
project(rqt_graphprofiler)

find_package(catkin REQUIRED COMPONENTS
#   message_generation
  genmsg
  std_msgs
)

catkin_python_setup()

## These msg types are for republishing visualized data, not implemented yet
# add_message_files(DIRECTORY msg
#   FILES
#   HostProfile.msg
#   NodeProfile.msg
#   TopicProfile.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

catkin_package()

install(PROGRAMS scripts/rqt_graphprofiler
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# catkin_add_nosetests(test)
