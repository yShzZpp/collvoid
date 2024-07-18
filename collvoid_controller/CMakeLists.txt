# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(collvoid_controller)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs tf geometry_msgs std_srvs collvoid_local_planner)

# catkin_python_setup()
catkin_package(
    DEPENDS rospy std_msgs tf geometry_msgs std_srvs collvoid_local_planner
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
catkin_install_python(PROGRAMS
  #所有的python文件install到bin目录下
	src/collvoid_controller/position_share_controller_candle.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
