cmake_minimum_required(VERSION 3.0.2)
project(ramp_475_46_16)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY urdf meshes config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
