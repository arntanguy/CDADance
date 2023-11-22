#include "MosquittoPlugin.h"

#include <mc_control/GlobalPluginMacros.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

namespace fs = std::filesystem;

namespace mc_plugin
{

constexpr int QOS = 1;

MosquittoPlugin::~MosquittoPlugin() {}

void MosquittoPlugin::init(mc_control::MCGlobalController &gc,
                           const mc_rtc::Configuration &config)
{
  auto &ctl = gc.controller();
  config("address", address_);
  config("port", port_);
  config("topic", topic_);
  config("client_id", client_id_);
  config("autoconnect", autoconnect_);
  config("publishEncoders", publishEncoders_);
  config("forceSensors", forceSensors_);
  persist_dir_ = config("persist_dir", std::string{fs::temp_directory_path() /
                                                   "mosquitto_persist"});
  mc_rtc::log::info("Persist dir: {}", persist_dir_);
  fs::create_directory(persist_dir_);
  // fs::remove_all(temp_dir); // delete the temp dir

  config("rate", rateDt_);
  iterRate_ = rateDt_ / gc.controller().timeStep;

  ctl.gui()->addElement(
      this, {"MosquittoPlugin"}, mc_rtc::gui::Input("Address", address_),
      mc_rtc::gui::Input("Port", port_), mc_rtc::gui::Input("Topic", topic_),
      mc_rtc::gui::Input("Client id", client_id_),
      mc_rtc::gui::Button("(Re)connect", [this]()
                          { connect(); }),
      mc_rtc::gui::Label("Connected", connected_));

  if (autoconnect_)
  {
    connect();
  }
}

void MosquittoPlugin::connect()
{
  connected_ = false;

  /* if(client_) */
  /* { // disconnect from existing client */
  /*   auto disconnect_token_ = client_->disconnect(1000); */
  /* } */

  std::string address = fmt::format("tcp://{}:{}", address_, port_);
  mc_rtc::log::info("Initializing for server '{}'...", address);
  client_ =
      std::make_unique<mqtt::async_client>(address, client_id_, persist_dir_);
  client_callback_ = std::make_unique<callback>(*this);
  client_->set_callback(*client_callback_);

  auto connOpts = mqtt::connect_options_builder().clean_session().finalize();

  try
  {
    mc_rtc::log::info("Connecting...");
    conn_token_ = client_->connect(connOpts);
  }
  catch (...)
  {
  }
}

void MosquittoPlugin::reset(mc_control::MCGlobalController &controller) {}

void MosquittoPlugin::before(mc_control::MCGlobalController &gc)
{
  auto &ctl = gc.controller();
  auto &robot = ctl.robot();

  if (!connected_)
  {  // Waiting for connection
    if (conn_token_ && conn_token_->is_complete())
    {
      mc_rtc::log::info("Connected");
      connected_ = true;
    }
    else
    {  // not connected yet
      return;
    }
  }

  if (iter_ == 0 || iter_ % iterRate_ == 0)
  {
    mc_rtc::Configuration msgConfig;
    std::map<std::string, double> encoders;
    std::map<std::string, double> encoders_normalized;
    const auto &outputRobot = ctl.outputRobot();

    if (publishEncoders_)
    {
      for (unsigned int i = 0; i < outputRobot.refJointOrder().size(); i++)
      {
        const auto &jName = outputRobot.refJointOrder()[i];
        const auto &jLimits = outputRobot.module().bounds();
        auto jInf = jLimits[0].at(jName)[0];
        auto jSup = jLimits[1].at(jName)[0];
        auto q = outputRobot.encoderValues()[i];
        auto jNormalized = std::fabs(q - jInf) / std::fabs(jSup - jInf);
        encoders[jName] = jNormalized;
        encoders_normalized[jName] = jNormalized;
      }
      msgConfig.add("encoders", encoders);
      msgConfig.add("encoders_normalized", encoders_normalized);
    }
    if (forceSensors_.size())
    {
      for (const auto &fs : forceSensors_)
      {
        if (robot.hasForceSensor(fs))
        {
          auto fsConf = msgConfig.add(fs);
          fsConf.add("force", robot.forceSensor(fs).force());
          fsConf.add("couple", robot.forceSensor(fs).force());
        }
      }
    }
    std::string msg = msgConfig.dump();
    if (!pub_token_)
    {
      try
      {
        pub_token_ =
            client_->publish(topic_, msg.data(), msg.size(), QOS, false);
      }
      catch (...)
      {
      }
    }
    else if (pub_token_ && pub_token_->is_complete())
    {
      pub_token_ = nullptr;
    }
  }
}

void MosquittoPlugin::after(mc_control::MCGlobalController &controller)
{
  ++iter_;
}

mc_control::GlobalPlugin::GlobalPluginConfiguration
MosquittoPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = false;
  return out;
}

}  // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("MosquittoPlugin", mc_plugin::MosquittoPlugin)
