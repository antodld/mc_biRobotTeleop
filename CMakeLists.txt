cmake_minimum_required(VERSION 3.2)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME BiRobotTeleoperation)
set(PROJECT_DESCRIPTION "BiRobotTeleoperation")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)

set(AROBASE "@")


foreach(CONFIG_FILE BiRobotTask)
  configure_file(etc/${CONFIG_FILE}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/${CONFIG_FILE}.yaml")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/${CONFIG_FILE}.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

  # add_definitions( -D_${CONFIG_FILE}_CONFIG_PATH=${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/${CONFIG_FILE} )
  set( CONFIG_PATH "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/${CONFIG_FILE}.yaml" )
  set( CONFIG_PATH_VAR "${CONFIG_FILE}_CONFIG_PATH")
  
  # add_definitions( -D${CONFIG_PATH_VAR}=${CONFIG_PATH} )

  message("HERE IS VAR ${CONFIG_PATH_VAR} ")
  message("HERE IS PATH ${CONFIG_PATH} ")

endforeach()



# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

configure_file(etc/BiRobotTeleoperation.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/BiRobotTeleoperation.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/BiRobotTeleoperation.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")