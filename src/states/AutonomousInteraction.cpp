#include "AutonomousInteraction.h"
#include "../mc_lipm_stabilizer.h"
#include <time.h>
#include <iostream>
#include <cmath>
#include <Python.h>

using namespace std;

void AutonomousInteraction::configure(const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("------ DEBUG enter AutonomousInteraction::configure ------\n");

  config("stiffness", stiffness_);
  config("robot", robot_);
  config("offset", offset_);

  mc_rtc::log::info("------ DEBUG leave AutonomousInteraction::configure ------\n");

}

void AutonomousInteraction::start(mc_control::fsm::Controller & ctl_)
{
    mc_rtc::log::info("------ DEBUG enter AutonomousInteraction::start ------\n");

    auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
    run(ctl);

    mc_rtc::log::info("------ DEBUG leave AutonomousInteraction::start ------\n");
}

bool AutonomousInteraction::run(mc_control::fsm::Controller & ctl_)
{
  mc_rtc::log::info("------ DEBUG enter AutonomousInteraction::run ------\n");

  auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
  auto & robot = ctl.robot(robot_);

  PyObject *pName = NULL;
  PyObject *pModule = NULL;
  PyObject *pFunc = NULL;
  PyObject *pValue = NULL;

  Py_Initialize();

  PyRun_SimpleString("import sys");
  PyRun_SimpleString("import os");
  PyRun_SimpleString("sys.path.append(os.getcwd())"); //in other words, when running simulation the dir need to be under {PROJECT}/src/states/ so that it could read *.py correctly
  PyRun_SimpleString("print(os.getcwd())");

  pModule = PyImport_ImportModule("hello_test_CPY");
  PyErr_Print();

  pFunc = PyObject_GetAttrString(pModule, "sayHello2");
  pValue = PyObject_CallObject(pFunc, NULL);

  Py_Finalize();

  // std::string SeqName[category_seq_] = {"SeqSF", "SeqAFL", "Seq3LookForApples", "Seq1Walk"};

  // srand( (unsigned)time(NULL) );
  // rand_val = 10 * (float) rand()/RAND_MAX;
  // remainder_val = int(floor(rand_val)) % category_seq_;

  // mc_rtc::log::info("------ Random Value = {}; output {} ------\n", remainder_val, SeqName[remainder_val]);
  // output(SeqName[remainder_val]);

  mc_rtc::log::info("------ DEBUG leave AutonomousInteraction::run ------\n");

  return true;
}

void AutonomousInteraction::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
}


EXPORT_SINGLE_STATE("AutonomousInteraction", AutonomousInteraction)