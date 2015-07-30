#pragma once
#include <yarp/os/RateThread.h>
#include <objectFeaturesThread.h>


namespace objectExploration
{
  class MaintainContactThread: public yarp::os::RateThread
  {
  public:

    MaintainContactThread(int period, ObjectFeaturesThread* objectFeatures):RateThread(period), _desiredForce(0),
    _objectFeatures(objectFeatures){};
    bool setDesiredForce(double desiredForce);
    void run();
    bool threadInit();
    void threadRelease();
  private:
    // things
    double _desiredForce;
    ObjectFeaturesThread* _objectFeatures;
  };
  
} // End of namespace