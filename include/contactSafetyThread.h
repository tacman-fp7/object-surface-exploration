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

    ContactSafetyThread(int period, ObjectFeaturesThread* objectFeatures, ICartesianControl* robotCartesianController):RateThread(period), _desiredForce(0),
        _objectFeatures(objectFeatures), _minDistalAngle(0), _dbgtag("Contact safety: "),
    _robotCartesianController(robotCartesianController){
        _collisionDetected = false;
    }
    void setDesiredForce(double desiredForce);
    void setMinDistalAngle(double minDistalAngle);
    void resetBaseline();
    void run();
    bool threadInit();
    void threadRelease();
    bool collisionDetected();

private:
    // things
    double _desiredForce;
    double _minDistalAngle;
    double _baseLine;
    ObjectFeaturesThread* _objectFeatures;
    std::string _dbgtag;
    ICartesianControl* _robotCartesianController;
    BufferedPort<Bottle> _forceTorque_in;
    BufferedPort<Bottle> _forceTorque_out;

    bool _collisionDetected;
};

} // namespace objectExploration
