add_fsm_state_simple(AutonomousInteraction)
if(TARGET mc_rtc::mc_rtc_ros)
add_fsm_state_simple(RosSubscriber)
target_link_libraries(RosSubscriber PUBLIC mc_rtc::mc_rtc_ros)
endif()

add_fsm_state_simple(InterpolatePosture)
add_fsm_state_simple(Walk)
add_fsm_state_simple(DisableWalk)

set(XSENS_DIR "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/${PROJECT_NAME}/xsens")
message(STATUS "XSENS DIR ${XSENS_DIR}")
set(AROBASE "@")
configure_file(data/xsens.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/data/xsens.yaml)
configure_file(data/interaction.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/data/interaction.yaml)
configure_file(data/interpolateposture.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/data/interpolateposture.yaml)
configure_file(data/HalfSitting.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/data/HalfSitting.yaml)

add_fsm_data_directory(${CMAKE_CURRENT_BINARY_DIR}/data)
