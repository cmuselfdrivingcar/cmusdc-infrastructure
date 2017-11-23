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


if(trajectory_prediction_CONFIG_INCLUDED)
  return()
endif()
set(trajectory_prediction_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(trajectory_prediction_SOURCE_PREFIX /home/teame16/CMUSelfDrivingCar/src/prediction/trajectory_prediction)
  set(trajectory_prediction_DEVEL_PREFIX /home/teame16/CMUSelfDrivingCar/devel)
  set(trajectory_prediction_INSTALL_PREFIX "")
  set(trajectory_prediction_PREFIX ${trajectory_prediction_DEVEL_PREFIX})
else()
  set(trajectory_prediction_SOURCE_PREFIX "")
  set(trajectory_prediction_DEVEL_PREFIX "")
  set(trajectory_prediction_INSTALL_PREFIX /home/teame16/CMUSelfDrivingCar/install)
  set(trajectory_prediction_PREFIX ${trajectory_prediction_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'trajectory_prediction' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(trajectory_prediction_FOUND_CATKIN_PROJECT TRUE)

if(NOT " " STREQUAL " ")
  set(trajectory_prediction_INCLUDE_DIRS "")
  set(_include_dirs "")
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${trajectory_prediction_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'trajectory_prediction' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  Ask the maintainer 'teame16 <teame16@todo.todo>' to fix it.")
      endif()
    else()
      message(FATAL_ERROR "Project 'trajectory_prediction' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/teame16/CMUSelfDrivingCar/install/${idir}'.  Ask the maintainer 'teame16 <teame16@todo.todo>' to fix it.")
    endif()
    _list_append_unique(trajectory_prediction_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND trajectory_prediction_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND trajectory_prediction_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND trajectory_prediction_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/teame16/CMUSelfDrivingCar/install/lib;/home/teame16/CMUSelfDrivingCar/devel/lib;/opt/ros/kinetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(trajectory_prediction_LIBRARY_DIRS ${lib_path})
      list(APPEND trajectory_prediction_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'trajectory_prediction'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND trajectory_prediction_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(trajectory_prediction_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${trajectory_prediction_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 trajectory_prediction_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${trajectory_prediction_dep}_FOUND)
      find_package(${trajectory_prediction_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${trajectory_prediction_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(trajectory_prediction_INCLUDE_DIRS ${${trajectory_prediction_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(trajectory_prediction_LIBRARIES ${trajectory_prediction_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${trajectory_prediction_dep}_LIBRARIES})
  _list_append_deduplicate(trajectory_prediction_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(trajectory_prediction_LIBRARIES ${trajectory_prediction_LIBRARIES})

  _list_append_unique(trajectory_prediction_LIBRARY_DIRS ${${trajectory_prediction_dep}_LIBRARY_DIRS})
  list(APPEND trajectory_prediction_EXPORTED_TARGETS ${${trajectory_prediction_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "trajectory_prediction-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${trajectory_prediction_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
