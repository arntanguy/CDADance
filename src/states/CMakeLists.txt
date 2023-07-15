# add_fsm_state_simple(AutonomousInteraction)

set(states_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/AutonomousInteraction.cpp
)

set(states_HDR
  ${CMAKE_CURRENT_SOURCE_DIR}/AutonomousInteraction.h
)

set(state_name AutonomousInteraction)

find_package(PythonLibs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

message(STATUS "PYTHON_LIBRARIES: ${PYTHON_LIBRARIES}")
message(STATUS "Python3_INCLUDE_DIRS : ${Python3_INCLUDE_DIRS}")
# add_executable(${PROJECT_NAME} main.cpp)

# The following are the decompose result of "add_fsm_state_simple" define by macro in the /home/dell/devel/src/mc_rtc/src/mc_rtcMacros.in.cmake
add_library(${state_name} SHARED ${states_SRC} ${states_HDR})

set_target_properties(${state_name} PROPERTIES PREFIX "")
set_target_properties(${state_name} 
    PROPERTIES 
        COMPILE_FLAGS "-DMC_CONTROL_FSM_STATE_EXPORTS"
)
set_target_properties(${state_name}
    PROPERTIES
        INSTALL_RPATH
        "${MC_STATES_DEFAULT_RUNTIME_INSTALL_PREFIX};${MC_STATES_RUNTIME_INSTALL_PREFIX}"
)
if(DEFINED CATKIN_DEVEL_PREFIX)
set_target_properties(${state_name} 
    PROPERTIES 
        LIBRARY_OUTPUT_DIRECTORY
        "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/states"
)
endif()


target_include_directories(${state_name}
    PRIVATE  
        ${Python3_INCLUDE_DIRS}
)
target_link_libraries(${state_name} PRIVATE Python3::Python)
target_link_libraries(${state_name} PUBLIC mc_rtc::mc_control_fsm)
# if(TARGET ${PROJECT_NAME})
# target_link_libraries(${state_name} PUBLIC ${PROJECT_NAME})
# endif()

install(
TARGETS ${state_name}
ARCHIVE
    DESTINATION
    "${MC_STATES_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
LIBRARY
    DESTINATION
    "${MC_STATES_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
RUNTIME
    DESTINATION
    "${MC_STATES_RUNTIME_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
)