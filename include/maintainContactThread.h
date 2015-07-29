#pragma once
#include <yarp/os/RateThread.h>

namespace objectExploration
{
  class MaintainContactThread: public yarp::os::RateThread
  {
  public:

    MaintainContactThread(int period):RateThread(period), _desiredForce(0){};
    bool setDesiredForce(double desiredForce);
    void run();
    bool threadInit();
    void threadRelease();
  private:
    // things
    double _desiredForce;
  };
  
} // End of namespace