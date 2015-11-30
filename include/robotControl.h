// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_robotControl
#define YARP_THRIFT_GENERATOR_robotControl

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class robotControl;


class robotControl : public yarp::os::Wire {
public:
  robotControl();
  virtual bool setHomePose();
  virtual bool goToHomePose();
  virtual bool setStartingPose();
  virtual bool goToStartingPose();
  virtual bool setEndPose();
  virtual bool goToEndPose();
  virtual bool startExploring();
  virtual bool stopExploring();
  virtual bool fingerSetAngle(const double angle);
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

