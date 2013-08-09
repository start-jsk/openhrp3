# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(openhrp3)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS openrtm_aist openrtm_aist_python)

# Build OpenHRP3
set(ENV{PATH} "$ENV{PATH}:${openrtm_aist_SOURCE_DIR}/bin")
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.openhrp3 installed
#                COMMAND cmake -E copy_directory ${PROJECT_SOURCE_DIR}/lib ${CATKIN_DEVEL_PREFIX}/lib # force copy under devel for catkin_package
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of OpenHRP3 failed")
endif(_make_failed)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

# fake add_library for catkin_package
add_library(hrpModel-3.1     SHARED IMPORTED)
add_library(hrpCollision-3.1 SHARED IMPORTED)
add_library(hrpUtil-3.1      SHARED IMPORTED)
set_target_properties(hrpModel-3.1     PROPERTIES IMPORTED_IMPLIB ${PROJECT_SOURCE_DIR}/lib/libhrpModel-3.1.so)
set_target_properties(hrpCollision-3.1 PROPERTIES IMPORTED_IMPLIB ${PROJECT_SOURCE_DIR}/lib/libhrpCollision-3.1.so)
set_target_properties(hrpUtil-3.1      PROPERTIES IMPORTED_IMPLIB ${PROJECT_SOURCE_DIR}/lib/libhrpUtil-3.1.so)

# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS eigen atlas f2c boost collada-dom
    CATKIN-DEPENDS openrtm_aist openrtm_aist_python
    INCLUDE_DIRS include/OpenHRP-3.1
    LIBRARIES hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1
)

# bin goes lib/openhrp3 so that it can be invoked from rosrun
install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS  # set executable
)
install(DIRECTORY lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY share
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



