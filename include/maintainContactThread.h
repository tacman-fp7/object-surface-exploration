#pragma once
#include <yarp/os/RateThread.h>

namespace objectExploration
{
  class MaintainContactThread: public yarp::os::RateThread
  {
  public:

    MaintainContactThread(int period):RateThread(period){};
    bool setDesiredForce(double desiredForce){/*Do nothing at the moment*/};
    void run(){/*Fill later*/};
  private:
    // things
  };
  
} // End of namespace