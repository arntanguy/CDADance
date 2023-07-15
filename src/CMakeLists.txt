set(controller_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_lipm_stabilizer.cpp
)

set(controller_HDR
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_lipm_stabilizer.h
)

# add_fsm_controller(lipm_stabilizer ${controller_SRC} ${controller_HDR})

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DMyFirstAdmittance_EXPORTS")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

# target_link_libraries(${PROJECT_NAME} PUBLIC ${PYTHON_LIBRARIES})
# include_directories(${PYTHON_INCLUDE_DIRS})
# include_directories(path/to/pyhelper)


# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find_package(Python COMPONENTS Interpreter Development)
# target_link_directories(${PROJECT_NAME}  PUBLIC ${Python_LIBRARY_DIRS})

# cmake_path(GET Python_EXECUTABLE PARENT_PATH Python_BIN_DIR) 
# set(config_cmd_flags "--cflags")
# execute_process(COMMAND ${Python_BIN_DIR}/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}-config ${config_cmd_flags} 
#   OUTPUT_VARIABLE Python_FLAGS)

# target_link_libraries(${PROJECT_NAME}  PUBLIC python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR})
# target_include_directories(${PROJECT_NAME}  PUBLIC ${Python_INCLUDE_DIRS})

# set_target_properties(python_extension PROPERTIES 
#   OUTPUT_NAME "call"
#   RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}
#   )
# separate_arguments(Python_FLAGS_NORM UNIX_COMMAND "${Python_FLAGS}")
# target_compile_options(python_extension PUBLIC ${Python_FLAGS_NORM})
add_subdirectory(states)