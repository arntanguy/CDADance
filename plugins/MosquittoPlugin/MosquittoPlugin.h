/*
 * Copyright 2021 CNRS-UM LIRMM
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mqtt/async_client.h>

#include <mutex>
#include <thread>

namespace mc_plugin
{

struct callback;

struct MosquittoPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController &controller,
            const mc_rtc::Configuration &config) override;

  void reset(mc_control::MCGlobalController &controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController &controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~MosquittoPlugin() override;

  bool connected() const noexcept { return connected_; }

  void connectionLost(const std::string &cause)
  {
    mc_rtc::log::error("[{}] Connection lost{}",
                       cause.size() ? " (cause: " + cause + ")" : "");
    connected_ = false;
  }

 protected:
  void connect();

 protected:
  std::string address_ = "127.0.0.1";
  uint port_ = 1883;
  std::string topic_ = "robot_state";
  std::string client_id_ = "mc_rtc_publisher";
  std::string persist_dir_;
  bool autoconnect_ = true;

  bool publishEncoders_ = true;
  std::vector<std::string> forceSensors_;

  std::unique_ptr<mqtt::async_client> client_;
  std::unique_ptr<callback> client_callback_;
  mqtt::token_ptr conn_token_ = nullptr;
  bool connected_ = false;
  mqtt::delivery_token_ptr pub_token_ = nullptr;

  double rateDt_ = 0.01;
  unsigned int iterRate_ = 0;
  unsigned int iter_ = 0;
};

/**
 * A callback class for use with the main MQTT client.
 */
struct callback : public virtual mqtt::callback
{
  callback(MosquittoPlugin &plugin) : plugin_(plugin) {}

  void connection_lost(const std::string &cause) override
  {
    plugin_.connectionLost(cause);
  }

  void delivery_complete(mqtt::delivery_token_ptr tok) override
  {
    /* mc_rtc::log::info("Delivery complete for token: {}", tok ?
     * tok->get_message_id() : -1); */
  }

 protected:
  MosquittoPlugin &plugin_;
};

}  // namespace mc_plugin
