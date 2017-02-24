#pragma once
#include <yarp/os/RateThread.h>
#include <objectFeaturesThread.h>
#include <yarp/dev/CartesianControl.h>
#include "hand.h"
#include <yarp/os/Mutex.h>

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
    yarp::os::Mutex _forceThreshold_mutex;
    double _forceThreshold;
    double _baseLine;
    std::string _dbgtag;
    Hand* _robotHand;
    BufferedPort<Bottle> _forceTorque_in;
    inline double getForceThreshold();

    bool _collisionDetected;
};

} // namespace objectExploration
