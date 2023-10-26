cmake_minimum_required(VERSION 3.10)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

set(PROJECT_NAME biRobotTeleoperation)
set(PROJECT_DESCRIPTION "")
set(PROJECT_URL "")
set(PROJECT_VERSION 0.0.1)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(INSTALL_FULL_INCLUDEDIR FALSE)
set(INSTALL_DOCUMENTATION OFF CACHE BOOL "")
set(INSTALL_GENERATED_HEADERS OFF CACHE BOOL "" FORCE)
set(INSTALL_PKG_CONFIG_FILE OFF CACHE BOOL "" FORCE)
set(CXX_DISABLE_WERROR ON)

#include(cmake/base.cmake)
include(insert_content.cmake)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

set(PACKAGE_EXTRA_MACROS "set(biRobotTeleoperation_STATES_LIBRARIES \"${MC_CONTROLLER_INSTALL_PREFIX}/biRobotTeleoperation/states\")
set(biRobotTeleoperation_STATES_FILES \"\${biRobotTeleoperation_STATES_LIBRARIES}/data\")")

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/BiRobotTeleoperation.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/BiRobotTeleoperation.yaml" @ONLY)
foreach(FILE BiRobotTasks DampingTasks HumanTasks RobotTeleopTasks ForcesDisplay JointsDamping)

  configure_file(etc/${FILE}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/${FILE}.yaml" @ONLY)
  insert_content(${CMAKE_CURRENT_BINARY_DIR}/etc/BiRobotTeleoperation.yaml ${CMAKE_CURRENT_BINARY_DIR}/etc/${FILE}.yaml ${FILE}.yaml)
endforeach()

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/BiRobotTeleoperation.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

