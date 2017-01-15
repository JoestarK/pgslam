cmake_minimum_required(VERSION 2.8)
project(pgslam)

set (CMAKE_BUILD_TYPE Release)

add_definitions (-std=c++11)
add_definitions (-DUSE_ISAM)

find_package (catkin REQUIRED COMPONENTS roscpp tf)

include_directories (include ${catkin_INCLUDE_DIRS})

catkin_package ()

include (FindPkgConfig)
pkg_check_modules (${PROJECT_NAME}_extra REQUIRED eigen3)
include_directories (${${PROJECT_NAME}_extra_INCLUDE_DIRS})

find_path (isam_INCLUDE_DIR isam.h /usr/include/isam /usr/local/include/isam)
find_library (isam_LIBRARY isam /usr/lib /usr/local/lib)
if (isam_INCLUDE_DIR MATCHES isam_INCLUDE_DIR-NOTFOUND OR isam_LIBRARY MATCHES isam_LIBRARY-NOTFOUND)
	message (FATAL_ERROR "please install isam first")
endif ()

add_executable (pgslam src/pgslam_node.cc src/pgslam.cc src/kdtree2d.cc)
target_link_libraries (pgslam ${catkin_LIBRARIES} isam cholmod)

install (TARGETS   pgslam DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install (DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install (DIRECTORY rviz   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install (DIRECTORY bag    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
