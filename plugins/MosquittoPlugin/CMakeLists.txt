find_package(PahoMqttCpp)

if(TARGET PahoMqttCpp::paho-mqttpp3)
  message(STATUS "Found PahoMqttCpp, building with MosquittoPlugin")
  set(AUTOLOAD_MosquittoPlugin_PLUGIN false CACHE BOOL "When true the plugin will always be loaded")

  set(plugin_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/MosquittoPlugin.cpp
  )

  set(plugin_HDR
    ${CMAKE_CURRENT_SOURCE_DIR}/MosquittoPlugin.h
  )
  add_plugin(MosquittoPlugin "${plugin_SRC}" "${plugin_HDR}")
  target_link_libraries(MosquittoPlugin PUBLIC PahoMqttCpp::paho-mqttpp3)
  target_include_directories(MosquittoPlugin PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
  install(FILES etc/MosquittoPlugin.yaml DESTINATION ${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc/)

  # add_executable(TestMosquittoPublisher ${CMAKE_CURRENT_SOURCE_DIR}/TestMosquittoPublisher.cpp)
  # target_link_libraries(TestMosquittoPublisher PUBLIC PahoMqttCpp::paho-mqttpp3)
else()
  message(STATUS "PahoMqttCpp not found, building without MosquittoPlugin support")
endif()
