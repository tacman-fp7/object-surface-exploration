#pragma once
#include <yarp/sig/Vector.h>
#include <yarp/os/RateThread.h>
#include <objectFeatures.h>

namespace objectExploration
{
  class ExplorationStrategyThread: public yarp::os::RateThread
  {
  public:

    ExplorationStrategyThread(int period):RateThread(period){};
    virtual void run(){};
    virtual yarp::sig::Vector getNextWayPoint(){/*Do nothing at the moment*/};
    
  private:
    ObjectFeatures* _objectFeatures;
    
  };
  
} // End of namespace