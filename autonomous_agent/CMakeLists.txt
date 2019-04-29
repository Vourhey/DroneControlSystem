cmake_minimum_required(VERSION 2.8.3)
project(autonomous_agent)

find_package(catkin REQUIRED COMPONENTS
  rospy
  drone_control_system
)

catkin_python_setup()

catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/trader_node
        scripts/worker_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/trader.launch
        launch/worker.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
