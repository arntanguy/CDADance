cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME CDADance)
set(PROJECT_DESCRIPTION "CDADance")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

option(CDADANCE_AUTO_MODE "Automatic demo" OFF)
option(CDADANCE_REAL_ROBOT_MODE "Set to ON to run on the real robot" OFF)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()
find_package(Eigen3 REQUIRED)
find_package(ismpc_walking QUIET)
find_package(lipm_walking_controller REQUIRED)
find_package(XsensPlugin REQUIRED)

if(TARGET ismpc_walking::ismpc_walking)
  message(STATUS "Found ISMPC Walking Controller, building with ISMPC support")
  set(WITH_ISMPC ON)
else()
  message(STATUS "ISMPC Walking Controller not found, building without ISMPC support")
  set(WITH_ISMPC OFF)
endif()

# Modes for the demo
if(${CDADANCE_AUTO_MODE})
  set(DEMO_TRANSITION "Auto")
else()
  set(DEMO_TRANSITION "Strict")
endif()

if(${CDADANCE_REAL_ROBOT_MODE})
  set(DEMO_MODE_COMMENT "#")
else()
  set(DEMO_MODE_COMMENT "")
endif()


add_subdirectory(src)
add_subdirectory(plugins)

set(XSENS_DIR "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/${PROJECT_NAME}/xsens")

install(
  FILES etc/datastore-to-replay.yaml
  DESTINATION ${XSENS_DIR}
)

install(
  DIRECTORY xsens
  DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/${PROJECT_NAME}
  FILES_MATCHING
  REGEX ".*(.bin$)"
)

configure_file(etc/CDADance.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/CDADance.yaml" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/CDADance.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/etc/mc_rtc_hrp4_real.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/etc/mc_rtc_hrp4_mujoco.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/etc/mc_rtc_rhps1_real.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/etc/mc_rtc_rhps1_mujoco.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

if(${WITH_ISMPC})
  configure_file(etc/CDADance.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/CDADance_ismpc.yaml")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/CDADance_ismpc.yaml" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")

  foreach(ROBOT hrp4)
    install_controller_robot_configuration("${PROJECT_NAME}_ismpc" etc/robots/${ROBOT}.yaml)
  endforeach()
endif()

install(FILES etc/MosquittoPlugin.yaml DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/CDADance/plugins/")
