#include "AutonomousInteraction.h"
#include "../mc_lipm_stabilizer.h"
#include <time.h>
#include <iostream>
#include <cmath>
#include <Python.h>
#define PY_SSIZE_T_CLEAN

using namespace std;

void AutonomousInteraction::configure(const mc_rtc::Configuration & config)
{
  if(debugmode_){mc_rtc::log::info("------ DEBUG enter AutonomousInteraction::configure ------\n");}

  config("stiffness", stiffness_);
  config("robot", robot_);
  config("offset", offset_);

  if(debugmode_){mc_rtc::log::info("------ DEBUG leave AutonomousInteraction::configure ------\n");}

}

void AutonomousInteraction::start(mc_control::fsm::Controller & ctl_)
{
    if(debugmode_){mc_rtc::log::info("------ DEBUG enter AutonomousInteraction::start ------\n");}

    auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
    run(ctl);

    if(debugmode_){mc_rtc::log::info("------ DEBUG leave AutonomousInteraction::start ------\n");}
}

bool AutonomousInteraction::run(mc_control::fsm::Controller & ctl_)
{
  if(debugmode_){mc_rtc::log::info("------ DEBUG enter AutonomousInteraction::run ------\n");}

  auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
  auto & robot = ctl.robot(robot_);
  
  // py::scoped_interpreter guard{}; // start the interpreter and keep it alive
  // py::print("Hello, World!");
  // auto np = py::module::import("hello_test_CPY");
  // mc_rtc::log::info("------ Successfully import python NUMPY ------\n");
  
  PyObject *pName = NULL;
  PyObject *pModule = NULL;
  PyObject *pFunc = NULL;
  PyObject *pValue = NULL;

  Py_Initialize();

  PyRun_SimpleString("import sys");
  PyRun_SimpleString("import os");
  PyRun_SimpleString("print(os.__file__)");
  PyRun_SimpleString("print(\"Python version\")");
  PyRun_SimpleString("print (sys.version)");
  PyRun_SimpleString("print(\"Version info.\")");
  PyRun_SimpleString("print (sys.version_info)");

  PyRun_SimpleString("sys.path.append(os.getcwd())"); //in other words, when running simulation the dir need to be under {PROJECT}/src/states/ so that it could read *.py correctly
  PyRun_SimpleString("print(sys.path)");

  // PyRun_SimpleString("sys.path.insert(0, os.path.abspath(\"/home/dell/.local/lib/python3.8/site-packages/\"))");
  // PyRun_SimpleString("print(sys.path)");

  const char* pyModuleName = "hello_test_CPY"; //hpe_recog2; hello_test_CPY
  const char* pyModuleCallFuncName = "sayHello2"; //human_pose_landmarks_estimation; sayHello2

  pModule = PyImport_ImportModule(pyModuleName);
  if(pModule != NULL)
  {
    mc_rtc::log::info("------ Successfully import python module ------\n");
  } 
  else
  {
    PyErr_Print();
  }
  
  pFunc = PyObject_GetAttrString(pModule, pyModuleCallFuncName);
  if(pFunc != NULL)
  {
    mc_rtc::log::info("------ Successfully import functions inside python module ------\n");
  } 
  else
  {
    PyErr_Print();
  }


  pValue = PyObject_CallObject(pFunc, NULL);

  Py_Finalize();

  std::string SeqName[category_seq_] = {"SeqSF", "SeqAFL", "Seq3LookForApples", "Seq1Walk"};
  int segIdx = static_cast<int>(PyLong_AsLong(pValue));

  mc_rtc::log::info("------ Return Index = {}; output = {} ------\n", segIdx, SeqName[segIdx]);
  output(SeqName[segIdx]);

  if(debugmode_){mc_rtc::log::info("------ DEBUG leave AutonomousInteraction::run ------\n");}

  return true;
}

void AutonomousInteraction::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
}


EXPORT_SINGLE_STATE("AutonomousInteraction", AutonomousInteraction)