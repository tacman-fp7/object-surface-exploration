#pragma once
#include <objectFeatures.h>
#include <yarp/os/RateThread.h>

namespace objectExploration
{
  class ObjectClassifierThread: public yarp::os::RateThread
  {
  public:
  
    ObjectClassifierThread(int period):RateThread(period){};
    bool update(){/*Do nothing at the moment*/};
    void rund(){};
  private:
    // A container for the features
    ObjectFeatures* _objectFeatures;
  };
  
} // End of namespace