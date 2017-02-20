#pragma once
#include <yarp/os/RateThread.h>
#include <objectFeaturesThread.h>
#include <yarp/dev/CartesianControl.h>
#include "hand.h"

namespace objectExploration
{
using yarp::dev::ICartesianControl;
class ContactSafetyThread: public yarp::os::RateThread
{
public:

    ContactSafetyThread(int period, Hand* robotHand);
    void setForceThreshold(double desiredForceThreshold);
    bool resetBaseline();
    void run();
    bool threadInit();
    bool collisionDetected();

private:
    // things
    double _forceThreshold;
    double _baseLine;
    std::string _dbgtag;
    Hand* _robotHand;
    BufferedPort<Bottle> _forceTorque_in;

    bool _collisionDetected;
};

} // namespace objectExploration
