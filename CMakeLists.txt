# toplevel CMakeLists.txt for a ros2 workspace ros2/cmake/toplevel.cmake

cmake_minimum_required(VERSION 3.0.2)

project(Project)

set(CATKIN_TOPLEVEL
    TRUE
)
set(CMAKE_PREFIX_PATH
    /opt/ros/noetic
)

# search for ros2 within the workspace
set(_cmd
    "ros2_find_pkg" "ros2" "${CMAKE_SOURCE_DIR}"
)
execute_process(
    COMMAND ${_cmd}
    RESULT_VARIABLE _res
    OUTPUT_VARIABLE _out
    ERROR_VARIABLE _err
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE
)
if(NOT _res EQUAL 0 AND NOT _res EQUAL 2)
    # searching fot ros2 resulted in an error
    string(REPLACE ";" " " _cmd_str "${_cmd}")
    message(
        FATAL_ERROR
            "Search for 'ros2' in workspace failed (${_cmd_str}): ${_err}"
    )
endif()

# include ros2 from workspace or via find_package()
if(_res EQUAL 0)
    set(ros2_EXTRAS_DIR "${CMAKE_SOURCE_DIR}/${_out}/cmake")
    # include all.cmake without add_subdirectory to let it operate in same scope
    include(${ros2_EXTRAS_DIR}/all.cmake NO_POLICY_SCOPE)
    add_subdirectory("${_out}")

else()
    # use either CMAKE_PREFIX_PATH explicitly passed to CMake as a command line
    # argument or CMAKE_PREFIX_PATH from the environment
    if(NOT DEFINED CMAKE_PREFIX_PATH)
        if(NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
            if(NOT WIN32)
                string(REPLACE ":" ";" CMAKE_PREFIX_PATH
                               $ENV{CMAKE_PREFIX_PATH}
                )
            else()
                set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
            endif()
        endif()
    endif()

    # list of ros2 workspaces
    set(ros2_search_path "")
    foreach(path ${CMAKE_PREFIX_PATH})
        if(EXISTS "${path}/.ros2")
            list(FIND ros2_search_path ${path} _index)
            if(_index EQUAL -1)
                list(APPEND ros2_search_path ${path})
            endif()
        endif()
    endforeach()

    # search for ros2 in all workspaces
    set(CATKIN_TOPLEVEL_FIND_PACKAGE TRUE)
    find_package(
        ros2 QUIET NO_POLICY_SCOPE PATHS ${ros2_search_path}
               NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
    )
    unset(CATKIN_TOPLEVEL_FIND_PACKAGE)

    if(NOT ros2_FOUND)
        message(
            FATAL_ERROR
                "find_package(ros2) failed. ros2 was neither found in the workspace nor in the CMAKE_PREFIX_PATH. One reason may be that no ROS setup.sh was sourced before."
        )
    endif()
endif()

ros2_workspace()
