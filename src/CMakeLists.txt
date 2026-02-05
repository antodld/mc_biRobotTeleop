set(CONFIG_PATH "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/BiRobotTask.yaml")
set(BiRobotTask_CONFIG_PATH ${CONFIG_PATH})
configure_file(yaml_path.in.h ${CMAKE_CURRENT_SOURCE_DIR}/yaml_path.h @ONLY)
configure_file(config.in.h "${CMAKE_BINARY_DIR}/include/config.h" @ONLY)

set(controller_SRC BiRobotTeleoperation.cpp HumanPoseEstimationJob.cpp)
set(controller_HDR BiRobotTeleoperation.h HumanPoseEstimationJob.h)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME}
                      PROPERTIES COMPILE_FLAGS "-DBiRobotTeleoperation_EXPORTS")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm)
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control
                                             mc_rtc::mc_rtc_ros)
target_link_libraries(${PROJECT_NAME}
                      PUBLIC biRobotTeleop::mc_tasks_biRobotTeleop)
target_link_libraries(${PROJECT_NAME} PUBLIC biRobotTeleop::utils_biRobotTeleop)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
         $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
         $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME} DESTINATION ${MC_RTC_LIBDIR})

add_controller(${PROJECT_NAME}_controller lib.cpp "")
set_target_properties(${PROJECT_NAME}_controller PROPERTIES OUTPUT_NAME
                                                            "${PROJECT_NAME}")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT "${PROJECT_NAME}Config"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(
  EXPORT "${PROJECT_NAME}Config"
  FILE "${PROJECT_NAME}Config.cmake"
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME})
