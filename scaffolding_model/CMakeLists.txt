cmake_minimum_required(VERSION 2.8.3)
project(uoe_lab_resources)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY urdf/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf/)
install(DIRECTORY meshes/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/)
