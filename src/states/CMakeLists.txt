add_fsm_state_simple(AutonomousInteraction)

# Build ROS states only for ros1
# TODO: support ros2
find_package(ament_cmake QUIET)
if(TARGET mc_rtc::mc_rtc_ros AND NOT ${ament_cmake_FOUND})
add_fsm_state_simple(RosSubscriber)
target_link_libraries(RosSubscriber PUBLIC mc_rtc::mc_rtc_ros)
endif()

add_fsm_state_simple(InterpolatePosture)
add_fsm_state_simple(Walk)
add_fsm_state_simple(DisableWalk)

set(XSENS_DIR "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/${PROJECT_NAME}/xsens")
message(STATUS "XSENS DIR ${XSENS_DIR}")

configure_file(data/xsens.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/data/xsens.yaml)
configure_file(data/interaction.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/data/interaction.yaml)
configure_file(data/interpolateposture.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/data/interpolateposture.yaml)
configure_file(data/HalfSitting.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/data/HalfSitting.yaml)

add_fsm_data_directory(${CMAKE_CURRENT_BINARY_DIR}/data)
