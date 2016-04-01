cmake_minimum_required(VERSION 2.8.3)
project(rqt_graph)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/dotcode_test.py)
endif()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_graph
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
