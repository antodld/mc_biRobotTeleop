add_fsm_state_simple(BiRobotTeleoperation_Initial)
add_fsm_state_simple(BiRobotTeleoperation_Task)
add_fsm_state_simple(HandDamping)
add_fsm_state_simple(JointsDamping)
add_fsm_state_simple(RelativePose)
add_fsm_state_simple(ForceDisplay)
add_fsm_state_simple(ChooseTransition)
add_fsm_state_simple(HumanPoseEstimation)

target_link_libraries(HumanPoseEstimation PUBLIC ${PROJECT_NAME})
target_link_libraries(RelativePose PUBLIC ${PROJECT_NAME})
target_link_libraries(JointsDamping PUBLIC ${PROJECT_NAME})
target_link_libraries(HandDamping PUBLIC ${PROJECT_NAME})
target_link_libraries(ForceDisplay PUBLIC ${PROJECT_NAME})
target_link_libraries(BiRobotTeleoperation_Task PUBLIC ${PROJECT_NAME})
target_link_libraries(BiRobotTeleoperation_Initial PUBLIC ${PROJECT_NAME})


add_fsm_data_directory(data)
