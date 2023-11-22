#
# Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(plugin_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/XsensReplay.cpp")
set(plugin_HDR "${CMAKE_CURRENT_SOURCE_DIR}/src/XsensReplay.h")

set(AUTOLOAD_XsensReplay_PLUGIN
    OFF
    CACHE INTERNAL "Automatically load Replay plugin"
)
add_plugin(XsensReplay ${plugin_SRC} ${plugin_HDR})
