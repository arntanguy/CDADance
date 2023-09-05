cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME LIPMStabilizer)
set(PROJECT_DESCRIPTION "LIPMStabilizer")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()
find_package(Eigen3 REQUIRED)

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/LIPMStabilizerWinnie.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMStabilizerWinnie.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMStabilizerWinnie.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
