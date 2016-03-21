#include <contactSafetyThread.h>
#include <stdio.h>
#include <iostream>
#include <yarp/sig/Vector.h>
#include <yarp/os/Network.h>
#include <math.h>

using std::cerr;
using std::cout;
using std::endl;
using yarp::sig::Vector;

namespace objectExploration
{


void ContactSafetyThread::run()
{

    Bottle* forceTorqueData = _forceTorque_in.read(true);

    if(!forceTorqueData->isNull())
    {
        double resultant = sqrt(
                    pow(forceTorqueData->get(0).asDouble(), 2) +
                    pow(forceTorqueData->get(1).asDouble(), 2) +
                    pow(forceTorqueData->get(2).asDouble(), 2));
        //cout << "FT: " << resultant << " BL: "  << _baseLine << endl;

        if(fabs(resultant - _baseLine) > 5){
            _robotCartesianController->stopControl();
            _collisionDetected = true;
            //cout << "Force toque exceeded" << endl;

        }
    }

    /* // Read the tactile data
    double contactForce;
    double distalAngle;
    Vector indexFingerAngels;



    contactForce = _objectFeatures->getContactForce();
    if(_objectFeatures->getIndexFingerAngles(indexFingerAngels))
        distalAngle = indexFingerAngels[2]; // The distal angle
    else
        cerr << _dbgtag << "failed to read the distal angle." << endl;
*/
    // Here do something smart with it.

    //cout << "Distal angle: " << distalAngle;
    //cout << " Contact froce: " << contactForce << endl;

    //if(contactForce > 2)
    //   _robotCartesianController->stopControl();

    //if(distalAngle < 1)
    //    _robotCartesianController->stopControl();

}



void ContactSafetyThread::resetBaseline()
{

     Bottle* forceTorqueData = _forceTorque_in.read(true);
    if(!forceTorqueData->isNull())
    {
        _baseLine = sqrt(
                    pow(forceTorqueData->get(0).asDouble(), 2) +
                    pow(forceTorqueData->get(1).asDouble(), 2) +
                    pow(forceTorqueData->get(2).asDouble(), 2));
        //cout << "FT: " << resultant << endl;

    }
    _collisionDetected = false;
}

bool ContactSafetyThread::collisionDetected()
{
    return _collisionDetected;
}

void ContactSafetyThread::setMinDistalAngle(double minDistalAngle)
{
    _minDistalAngle = minDistalAngle;
}

void ContactSafetyThread::setDesiredForce(double desiredForce)
{
    _desiredForce = desiredForce;
}



bool ContactSafetyThread::threadInit()
{
    yarp::os::RateThread::threadInit();

    if(_objectFeatures == NULL){
        cout << _dbgtag << " failed to initialise -- objectFeatures points to NULL, aborting" << endl;
        return false;
    }
    else
    {
        cout << "ContactSafetyThread configured" << endl;
    }

    /// Force torque//////

    if(_forceTorque_in.open("/safeConact/" + _objectFeatures->getArm() + "_arm/forceTorque/analog:i"))
    {
        yarp::os::Network::connect( "/" + _objectFeatures->getRobotName() + "/" + _objectFeatures->getArm() + "_arm/analog:o",
                                    "/safeConact/" + _objectFeatures->getArm() + "_arm/forceTorque/analog:i");
    }

    _forceTorque_out.open("/safeContact/" + _objectFeatures->getArm() + "_arm/forceTorque/resultant:o");


    Bottle* forceTorqueData = _forceTorque_in.read(true);

    resetBaseline();

    ///icub/left_arm/analog:o
    return true;
}


void ContactSafetyThread::threadRelease()
{
    yarp::os::RateThread::threadRelease();
}

} // namespace objectExploration
