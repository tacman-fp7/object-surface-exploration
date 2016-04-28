#pragma once
#include <yarp/os/RateThread.h>
#include <objectFeaturesThread.h>
#include <yarp/dev/CartesianControl.h>

namespace objectExploration
{
using yarp::dev::ICartesianControl;
class ContactSafetyThread: public yarp::os::RateThread
{
public:

    ContactSafetyThread(int period, ICartesianControl* robotCartesianController);
    void setForceThreshold(double desiredForceThreshold);
    bool resetBaseline();
    void run();
    bool threadInit();
    //void threadRelease();
    bool collisionDetected();
private:
    void init();
private:
    // things
    double _forceThreshold;
    //double _minDistalAngle;
    double _baseLine;
    ObjectFeaturesThread* _objectFeatures;
    std::string _dbgtag;
    ICartesianControl* _robotCartesianController;
    BufferedPort<Bottle> _forceTorque_in;

    bool _collisionDetected;
};

} // namespace objectExploration
