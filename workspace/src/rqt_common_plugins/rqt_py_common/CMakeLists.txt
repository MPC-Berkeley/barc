cmake_minimum_required(VERSION 2.8.3)
project(rqt_py_common)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED)
catkin_python_setup()

if(CATKIN_ENABLE_TESTING)
  find_package(genmsg REQUIRED)
  find_package(std_msgs REQUIRED)
  add_message_files(
    DIRECTORY test/msg
    FILES ArrayVal.msg Val.msg
    NOINSTALL
    )
  generate_messages(DEPENDENCIES std_msgs)
endif()

catkin_package()

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()