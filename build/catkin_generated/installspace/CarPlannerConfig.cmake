# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(CarPlanner_CONFIG_INCLUDED)
  return()
endif()
set(CarPlanner_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(CarPlanner_SOURCE_PREFIX /home/mike/code/CarPlanner)
  set(CarPlanner_DEVEL_PREFIX /home/mike/code/CarPlanner/build/devel)
  set(CarPlanner_INSTALL_PREFIX "")
  set(CarPlanner_PREFIX ${CarPlanner_DEVEL_PREFIX})
else()
  set(CarPlanner_SOURCE_PREFIX "")
  set(CarPlanner_DEVEL_PREFIX "")
  set(CarPlanner_INSTALL_PREFIX /usr/local)
  set(CarPlanner_PREFIX ${CarPlanner_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'CarPlanner' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(CarPlanner_FOUND_CATKIN_PROJECT TRUE)

if(NOT "include;/usr/local/include/eigen3;/home/mike/code/Sophus/include;/home/mike/code/Sophus/build/include;/home/mike/code/CVars/include;/home/mike/code/CVars/build/include;/usr/local/include/bullet;/usr/include/assimp;/usr/include;/usr/include/IL;/usr/include/IL/..;/usr/include/x86_64-linux-gnu;/home/mike/code/Pangolin/include;/home/mike/code/Pangolin/build/include;/home/mike/code/SceneGraph/include;/home/mike/code/SceneGraph/build/include;/home/mike/code/HAL;/home/mike/code/HAL/build;/home/mike/catkin_ws/devel/.private/carplanner_msgs/include;/opt/ros/melodic/include;/opt/ros/melodic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp " STREQUAL " ")
  set(CarPlanner_INCLUDE_DIRS "")
  set(_include_dirs "include;/usr/local/include/eigen3;/home/mike/code/Sophus/include;/home/mike/code/Sophus/build/include;/home/mike/code/CVars/include;/home/mike/code/CVars/build/include;/usr/local/include/bullet;/usr/include/assimp;/usr/include;/usr/include/IL;/usr/include/IL/..;/usr/include/x86_64-linux-gnu;/home/mike/code/Pangolin/include;/home/mike/code/Pangolin/build/include;/home/mike/code/SceneGraph/include;/home/mike/code/SceneGraph/build/include;/home/mike/code/HAL;/home/mike/code/HAL/build;/home/mike/catkin_ws/devel/.private/carplanner_msgs/include;/opt/ros/melodic/include;/opt/ros/melodic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'mike <mike@todo.todo>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${CarPlanner_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'CarPlanner' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'CarPlanner' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/usr/local/${idir}'.  ${_report}")
    endif()
    _list_append_unique(CarPlanner_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "/usr/local/lib/libBulletDynamics.so;/usr/local/lib/libBulletCollision.so;/usr/local/lib/libLinearMath.so;/usr/local/lib/libBulletSoftBody.so;/usr/lib/x86_64-linux-gnu/libboost_timer.so;/usr/lib/x86_64-linux-gnu/libGL.so;/usr/lib/x86_64-linux-gnu/libGLU.so;/usr/lib/x86_64-linux-gnu/libGLEW.so;/usr/lib/x86_64-linux-gnu/libIL.so;/usr/lib/x86_64-linux-gnu/libILU.so;/usr/lib/x86_64-linux-gnu/libILUT.so;/usr/lib/x86_64-linux-gnu/libpng.so;/usr/lib/x86_64-linux-gnu/libjpeg.so;/usr/lib/x86_64-linux-gnu/libtiff.so;/usr/lib/x86_64-linux-gnu/libglut.so;/usr/lib/x86_64-linux-gnu/libz.so;/home/mike/code/Pangolin/build/libpangolin.so;/home/mike/code/SceneGraph/build/libscenegraph.so;/home/mike/code/CVars/build/libcvars.so;/usr/lib/x86_64-linux-gnu/libprotobuf.so;/usr/lib/x86_64-linux-gnu/libzmq.so;/usr/lib/x86_64-linux-gnu/libzmqpp.so;/usr/local/lib/libglog.so;dl;/usr/lib/x86_64-linux-gnu/libdns_sd.so;/home/mike/code/Node/build/libnode.so;/usr/local/lib/libvrpn.a;/usr/local/lib/libquat.a;-lpthread;/usr/lib/x86_64-linux-gnu/libtinyxml2.so;/opt/ros/melodic/lib/libimage_transport.so;/opt/ros/melodic/lib/libclass_loader.so;/usr/lib/libPocoFoundation.so;/usr/lib/x86_64-linux-gnu/libdl.so;/opt/ros/melodic/lib/libroslib.so;/opt/ros/melodic/lib/librospack.so;/usr/lib/x86_64-linux-gnu/libpython2.7.so;/usr/lib/x86_64-linux-gnu/libboost_program_options.so;/usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0;/home/mike/code/HAL/build/HAL/libhal.so;/usr/lib/x86_64-linux-gnu/libassimp.so;/usr/lib/x86_64-linux-gnu/libyaml-cpp.so.0.5.2;/usr/local/lib/libgflags.so;/opt/ros/melodic/lib/libtf.so;/opt/ros/melodic/lib/libtf2_ros.so;/opt/ros/melodic/lib/libactionlib.so;/opt/ros/melodic/lib/libmessage_filters.so;/opt/ros/melodic/lib/libtf2.so;/opt/ros/melodic/lib/libroscpp.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_signals.so;/opt/ros/melodic/lib/librosconsole.so;/opt/ros/melodic/lib/librosconsole_log4cxx.so;/opt/ros/melodic/lib/librosconsole_backend_interface.so;/usr/lib/x86_64-linux-gnu/liblog4cxx.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;/opt/ros/melodic/lib/libxmlrpcpp.so;/opt/ros/melodic/lib/libroscpp_serialization.so;/opt/ros/melodic/lib/librostime.so;/opt/ros/melodic/lib/libcpp_common.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libpthread.so;/usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND CarPlanner_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND CarPlanner_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND CarPlanner_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND CarPlanner_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /usr/local/lib;/home/mike/catkin_ws/devel/lib;/opt/ros/melodic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(CarPlanner_LIBRARY_DIRS ${lib_path})
      list(APPEND CarPlanner_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'CarPlanner'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND CarPlanner_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(CarPlanner_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${CarPlanner_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "roscpp;std_msgs;message_runtime;nav_msgs;carplanner_msgs")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 CarPlanner_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${CarPlanner_dep}_FOUND)
      find_package(${CarPlanner_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${CarPlanner_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(CarPlanner_INCLUDE_DIRS ${${CarPlanner_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(CarPlanner_LIBRARIES ${CarPlanner_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${CarPlanner_dep}_LIBRARIES})
  _list_append_deduplicate(CarPlanner_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(CarPlanner_LIBRARIES ${CarPlanner_LIBRARIES})

  _list_append_unique(CarPlanner_LIBRARY_DIRS ${${CarPlanner_dep}_LIBRARY_DIRS})
  list(APPEND CarPlanner_EXPORTED_TARGETS ${${CarPlanner_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${CarPlanner_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
