cmake_minimum_required(VERSION 3.0.2)
project(30x30_small)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY urdf meshes config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
