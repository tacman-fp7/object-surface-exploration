#pragma once
#include <objectFeaturesThread.h>
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
    ObjectFeaturesThread* _objectFeatures;
};

} // namespace objectExploration
