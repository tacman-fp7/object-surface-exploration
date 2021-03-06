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
  virtual bool setStartingPose();
  virtual bool goToStartingPose();
  virtual bool setEndPose();
  virtual bool goToEndPose();
  virtual bool startExploring(const std::string& type, const std::string& objectName);
  virtual bool stopExploring();
  virtual bool fingerSetAngle(const double angle);
  virtual bool prepHand();
  virtual bool openHand();
  virtual bool calibrateHand();
  virtual bool enableSurfaceSampling();
  virtual bool disableSurfaceSampling();
  virtual bool refineModelEnable();
  virtual bool refineModelDisable();
  virtual bool nRepeatsSet(const int32_t nRepeats);
  virtual bool validatePositionsEnable();
  virtual bool validatePositionsDisable();
  virtual bool setHeight(const double height);
  virtual bool alignFingers();
  virtual bool setSafetyThreshold(const double threshold);
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
