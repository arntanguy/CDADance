#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/EndEffectorTask.h>

struct XsensBodyConfiguration
{
    std::string segmentName{};
    std::string bodyName{};
    sva::PTransformd offset = sva::PTransformd::Identity();
    double weight = 1000;
    double stiffness = 100;
};

struct XsensRetargetting : mc_control::fsm::State
{
    void start(mc_control::fsm::Controller & ctl) override;
    bool run(mc_control::fsm::Controller & ctl) override;
    void teardown(mc_control::fsm::Controller & ctl) override;
private:
    std::map<std::string, XsensBodyConfiguration> bodyConfigurations_;
    std::map<std::string, std::unique_ptr<mc_tasks::EndEffectorTask>> tasks_;
    std::vector<std::string> ignoreJointsAry; 
    double stiffness_ = 10;
    double weight_ = 1000;
    std::string robot_{};
    sva::PTransformd offset_ = sva::PTransformd::Identity();
    bool fixBaseLink_ = true;
    sva::PTransformd initPosW_ = sva::PTransformd::Identity();
    bool finished_ = false;
    std::vector<std::string> unactiveJoints_ = {};

    bool debugmode_ = true;
};
