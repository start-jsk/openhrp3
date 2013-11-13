# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(openhrp3)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS openrtm_aist_python)

# Build OpenHRP3
set(ENV{PKG_CONFIG_PATH} $ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig) # 1) set pkg-config path
find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED) # 2) invoke pkg-confg
set(ENV{PATH} $ENV{PATH}:${openrtm_aist_PREFIX}/lib/openrtm_aist/bin) # 3) get PREFIX and update PATH
execute_process(
  COMMAND sh -c "test -e ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME} || rm -f ${PROJECT_SOURCE_DIR}/installed ${PROJECT_SOURCE_DIR}/build/OpenHRP-3.1/CMakeCache.txt"
  COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.openhrp3 INSTALL_DIR=${CATKIN_DEVEL_PREFIX} installed
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Compile openhrp3 failed")
endif(_make_failed)

# binary files intentionally goes to ${CATKIN_PACKAGE_BIN_DESTINATION}/lib
execute_process(
  COMMAND sh -c "test -e ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME} || (mkdir -p ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}; mv ${CATKIN_DEVEL_PREFIX}/bin/* ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/)"
  RESULT_VARIABLE _make_failed
  OUTPUT_VARIABLE _copy_bin)
message("copy binary files ${_copy_bin}")
if (_make_failed)
  message(FATAL_ERROR "Copy openhrp3/bin failed: ${_make_failed}")
endif(_make_failed)

# shared files intentionally goes to ${CATKIN_PACKAGE_SHARE_DESTINATION}
execute_process(
  COMMAND sh -c "test -e ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/share/OpenHRP-3.1 || (mkdir -p ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/share; mv ${CATKIN_DEVEL_PREFIX}/share/OpenHRP-3.1 ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/share/)"
  RESULT_VARIABLE _make_failed
  OUTPUT_VARIABLE _copy_share)
message("copy shared files ${_copy_share}")
if (_make_failed)
  message(FATAL_ERROR "Copy openhrp3/share failed: ${_make_failed}")
endif(_make_failed)

# copy of shared files intentionally goes to ${PROJECT_SOURCE_DIR} for hrpsys_tools/launch/_gen_project.launch
execute_process(
  COMMAND sh -c "test -e ${PROJECT_SOURCE_DIR}/share/OpenHRP-3.1 || (mkdir -p ${PROJECT_SOURCE_DIR}/share/OpenHRP-3.1; cp -r ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/share/OpenHRP-3.1/ ${PROJECT_SOURCE_DIR}/share/)"
  RESULT_VARIABLE _make_failed
  OUTPUT_VARIABLE _copy_share)
message("copy shared files ${_copy_share}")
if (_make_failed)
  message(FATAL_ERROR "Copy openhrp3/share failed: ${_make_failed}")
endif(_make_failed)

# fix idl file location
execute_process(
  COMMAND sed -i s@{prefix}/share/OpenHRP-3.1@{prefix}/share/openhrp3/share/OpenHRP-3.1@g ${CATKIN_DEVEL_PREFIX}/lib/pkgconfig/openhrp3.1.pc
  RESULT_VARIABLE _make_failed
  OUTPUT_VARIABLE _fix_pc)
message("fix openhrp3.1.pc file ${_fix_pc}")
if (_make_failed)
  message(FATAL_ERROR "fix openhrp3.1.pc failed: ${_make_failed}")
endif(_make_failed)


# fake add_library for catkin_package
add_library(hrpModel-3.1     SHARED IMPORTED)
add_library(hrpCollision-3.1 SHARED IMPORTED)
add_library(hrpUtil-3.1      SHARED IMPORTED)
set_target_properties(hrpModel-3.1     PROPERTIES IMPORTED_IMPLIB ${CATKIN_DEVEL_PREFIX}/lib/libhrpModel-3.1.so)
set_target_properties(hrpCollision-3.1 PROPERTIES IMPORTED_IMPLIB ${CATKIN_DEVEL_PREFIX}/lib/libhrpCollision-3.1.so)
set_target_properties(hrpUtil-3.1      PROPERTIES IMPORTED_IMPLIB ${CATKIN_DEVEL_PREFIX}/lib/libhrpUtil-3.1.so)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/OpenHRP-3.1) # fake catkin_package
catkin_package(
    DEPENDS eigen atlas f2c boost collada-dom openrtm-aist
    CATKIN-DEPENDS openrtm_aist_python
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include/OpenHRP-3.1
    LIBRARIES hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1
    SKIP_CMAKE_CONFIG_GENERATION
    SKIP_PKG_CONFIG_GENERATION
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  USE_SOURCE_PERMISSIONS  # set executable
)
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/share/openhrp3/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(CODE
  "execute_process(COMMAND echo \"fix \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/pkgconfig/openhrp3.1.pc\")
   execute_process(COMMAND sed -i s@${CATKIN_DEVEL_PREFIX}@${CMAKE_INSTALL_PREFIX}@g \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/pkgconfig/openhrp3.1.pc) # basic
   execute_process(COMMAND sed -i s@{prefix}/include@{prefix}/include/openhrp3/include@g \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/pkgconfig/openhrp3.1.pc) # --cflags
")


