cmake_minimum_required(VERSION 3.2)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME BiRobotTeleoperation)
set(PROJECT_DESCRIPTION "BiRobotTeleoperation")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

include(insert_content.cmake)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)

set(AROBASE "@")

configure_file(etc/BiRobotTeleoperation.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/BiRobotTeleoperation.yaml" @ONLY)

foreach(FILE BiRobotTasks DampingTasks HumanTasks RobotTeleopTasks)

  configure_file(etc/${FILE}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/${FILE}.yaml" @ONLY)
  insert_content(${CMAKE_CURRENT_BINARY_DIR}/etc/BiRobotTeleoperation.yaml ${CMAKE_CURRENT_BINARY_DIR}/etc/${FILE}.yaml ${FILE}.yaml)

endforeach()

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/BiRobotTeleoperation.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")