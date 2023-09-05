
set( CONFIG_PATH "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/BiRobotTask.yaml" )
set(BiRobotTask_CONFIG_PATH ${CONFIG_PATH})
configure_file (yaml_path.in.h ${CMAKE_CURRENT_SOURCE_DIR}/yaml_path.h @ONLY)

set(controller_SRC
  BiRobotTeleoperation.cpp
)

set(controller_HDR
  BiRobotTeleoperation.h
)


add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DBiRobotTeleoperation_EXPORTS")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm)
target_link_libraries(${PROJECT_NAME} PUBLIC bilateralTeleop_dataLink)
target_link_libraries(${PROJECT_NAME} PUBLIC mc_tasks_biRobotTeleop)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)