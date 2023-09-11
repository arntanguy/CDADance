set(controller_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_lipm_stabilizer.cpp
)

set(controller_HDR
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_lipm_stabilizer.h
)

# add_fsm_controller(lipm_stabilizer ${controller_SRC} ${controller_HDR})

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DMyFirstAdmittance_EXPORTS")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm mc_rtc::mc_rtc_ros)
target_link_libraries(${PROJECT_NAME} PUBLIC lipm_walking_controller::lipm_walking_controller)
target_link_libraries(${PROJECT_NAME} PUBLIC ismpc_walking::ismpc_walking)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)
