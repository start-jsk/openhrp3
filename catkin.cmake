# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(openhrp3)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS mk rostest openrtm_aist)

# Build OpenHRP3
# <devel>/lib/<package>/bin/openrhrp-*
# <devel>/lib/libhrp...
# <src>/<package>/share
if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/installed)

  set(ENV{PATH} ${openrtm_aist_PREFIX}/lib/openrtm_aist/bin/:$ENV{PATH}) #update PATH for rtm-config
  execute_process(COMMAND cmake -E remove -f ${CMAKE_CURRENT_BINARY_DIR}/build/OpenHRP-3.1/CMakeCache.txt)
  execute_process(
    COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR}
    make -f ${PROJECT_SOURCE_DIR}/Makefile.openhrp3
    INSTALL_DIR=${CATKIN_DEVEL_PREFIX}
    MK_DIR=${mk_PREFIX}/share/mk
    PATCH_DIR=${PROJECT_SOURCE_DIR}
    installed
    RESULT_VARIABLE _make_failed)
  if (_make_failed)
    message(FATAL_ERROR "Compile openhrp3 failed")
  endif(_make_failed)

  # binary files intentionally goes to ${CATKIN_PACKAGE_BIN_DESTINATION}/lib
  ## make directory
  execute_process(
    COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}
    RESULT_VARIABLE _make_failed)
  if (_make_failed)
    message(FATAL_ERROR "make_directory ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME} failed: ${_make_failed}")
  endif(_make_failed)
  ## copy programs
  foreach(openhrp3_program export-collada openhrp-aist-dynamics-simulator openhrp-controller-bridge openhrp-model-loader  openhrp-shutdown-servers export-vrml openhrp-collision-detector openhrp-jython-prompt openhrp-path-planner openhrp-ut-dynamics-simulator)
    execute_process(
      COMMAND cmake -E rename ${CATKIN_DEVEL_PREFIX}/bin/${openhrp3_program} ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/${openhrp3_program}
      RESULT_VARIABLE _make_failed)
    message("move binary files ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/${openhrp3_program}")
    if (_make_failed)
      message(FATAL_ERROR "Move openhrp3/bin failed: ${_make_failed}")
    endif(_make_failed)
  endforeach()

  # fix openhrp3.pkg
  message("sed -i s@\${prefix}/share@${PROJECT_SOURCE_DIR}/share@g ${CATKIN_DEVEL_PREFIX}/lib/pkgconfig/openhrp3.1.pc")
  execute_process(
    COMMAND sed -i s@\${prefix}/share@${PROJECT_SOURCE_DIR}/share@g ${CATKIN_DEVEL_PREFIX}/lib/pkgconfig/openhrp3.1.pc
    OUTPUT_VARIABLE _sed_output
    RESULT_VARIABLE _sed_failed)
  if (_sed_failed)
    message(FATAL_ERROR "sed ${CATKIN_DEVEL_PREFIX}/lib/pkgconfig/openhrp3.1.pc failed: ${_make_failed}")
  endif(_sed_failed)

  # move share directory
  if(NOT EXISTS ${PROJECT_SOURCE_DIR}/share/OpenHRP-3.1/)
    execute_process(
      COMMAND cmake -E make_directory ${PROJECT_SOURCE_DIR}/share/OpenHRP-3.1/
      RESULT_VARIABLE _make_failed)
    if (_make_failed)
      message(FATAL_ERROR "make_directory ${PROJECT_SOURCE_DIR}/share/OpenHRP-3.1/ failed: ${_make_failed}")
    endif(_make_failed)
    execute_process(
      COMMAND cmake -E rename ${CATKIN_DEVEL_PREFIX}/share/OpenHRP-3.1/ ${PROJECT_SOURCE_DIR}/share/OpenHRP-3.1/
      RESULT_VARIABLE _make_failed)
    message("move share directory ${CATKIN_DEVEL_PREFIX}/share/OpenHRP-3.1/ ${PROJECT_SOURCE_DIR}/share/OpenHRP-3.1/")
    if (_make_failed)
      message(FATAL_ERROR "Move share/OpenHRP-3.1 failed: ${_make_failed}")
    endif(_make_failed)
  endif()

endif(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/installed)

##
## catkin package
##
catkin_package(
)

##
## install
##

install(
  DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/OpenHRP-3.1
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

# use install_manifest.xml to copy files under devel to install, not to overwrite existing files (ex, openrtm-aist.pc with path modification) by copying entire directory
### -> install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROEJCT_NAME})
execute_process(
  COMMAND grep ${CATKIN_DEVEL_PREFIX}/bin ${CMAKE_CURRENT_BINARY_DIR}/build/OpenHRP-3.1/install_manifest.txt
  OUTPUT_VARIABLE _bin_files RESULT_VARIABLE _grep_failed)
if (_grep_failed)
  message(FATAL_ERROR "grep bin ${CMAKE_CURRENT_BINARY_DIR}/build/OpenHRP-3.1/install_manifest.txt ${_make_failed}")
endif(_grep_failed)
execute_process(
  COMMAND grep ${CATKIN_DEVEL_PREFIX}/lib ${CMAKE_CURRENT_BINARY_DIR}/build/OpenHRP-3.1/install_manifest.txt
  OUTPUT_VARIABLE _lib_files RESULT_VARIABLE _grep_failed)
if (_grep_failed)
  message(FATAL_ERROR "grep lib ${CMAKE_CURRENT_BINARY_DIR}/build/OpenHRP-3.1/install_manifest.txt ${_make_failed}")
endif(_grep_failed)

string(REGEX REPLACE "\n" ";" _bin_files ${_bin_files})
string(REGEX REPLACE "\n" ";" _lib_files ${_lib_files})
foreach(_bin_file ${_bin_files})
  get_filename_component(_bin_file_name ${_bin_file} NAME)
  install(PROGRAMS ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/${_bin_file_name} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME})
endforeach()
foreach(_lib_file ${_lib_files})
  get_filename_component(_lib_file_path ${_lib_file} PATH)
  string(REPLACE "${CATKIN_DEVEL_PREFIX}/lib" "" _lib_file_path ${_lib_file_path})
  install(FILES ${_lib_file} DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}/${_lib_file_path})
endforeach()
## done copy libs

install(DIRECTORY test share
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(CODE
  "execute_process(COMMAND echo \"fix \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/pkgconfig/openhrp3.pc ${CATKIN_DEVEL_PREFIX} -> ${CMAKE_INSTALL_PREFIX}\")
   execute_process(COMMAND sed -i s@${CATKIN_DEVEL_PREFIX}@${CMAKE_INSTALL_PREFIX}@g \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/pkgconfig/openhrp3.pc) # basic
")

install(CODE
  "execute_process(COMMAND echo \"fix \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/pkgconfig/openhrp3.pc ${CATKIN_DEVEL_PREFIX} -> ${CMAKE_INSTALL_PREFIX}\")
   execute_process(COMMAND sed -i s@${CATKIN_DEVEL_PREFIX}@${CMAKE_INSTALL_PREFIX}@g \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/pkgconfig/openhrp3.1.pc) #
   execute_process(COMMAND sed -i s@${PROJECT_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/share/openhrp3@g \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/pkgconfig/openhrp3.1.pc) # basic
   execute_process(COMMAND sed -i s@{prefix}/bin@{prefix}/lib/openhrp3@g \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/pkgconfig/openhrp3.1.pc) # basic
   execute_process(COMMAND sed -i s@{prefix}/share@{prefix}/share/openhrp3/share@g \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/pkgconfig/openhrp3.1.pc) # basic
")

##
##
##
add_rostest(test/test_openhrp3.test)
add_rostest(test/test_modelloader.test)
