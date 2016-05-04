#include <contactSafetyThread.h>
#include <stdio.h>
#include <iostream>
#include <yarp/sig/Vector.h>
#include <yarp/os/Network.h>
#include <math.h>
//#include <yarp/os/ResourceFinder.h>

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
        // Get the resultant force
        double resultant = sqrt(
                    pow(forceTorqueData->get(0).asDouble(), 2) +
                    pow(forceTorqueData->get(1).asDouble(), 2) +
                    pow(forceTorqueData->get(2).asDouble(), 2));


        // If the resultant force is greater than the threshold stop the
        // arm movemnet
        if(fabs(resultant - _baseLine) > _forceThreshold){
            _robotHand->stopControl();
            _collisionDetected = true;

        }
    }
}



ContactSafetyThread::ContactSafetyThread(int period,  Hand *robotHand)
    :RateThread(period){

    _forceThreshold = 5;
    //_objectFeatures = objectFeatures;
    _dbgtag = "Contact safety: ";
    _robotHand = robotHand;
    _collisionDetected = false;

    init();



}

void ContactSafetyThread::init(){

    //ResourceFinder rf;

    /// Force torque//////


    if(_forceTorque_in.open("/safeConact/" + _robotHand->getArmName() + "_arm/forceTorque/analog:i"))
    {
        yarp::os::Network::connect( "/" + _robotHand->getRobotName() + "/" + _robotHand->getArmName() + "_arm/analog:o",
                                    "/safeConact/" + _robotHand->getArmName() + "_arm/forceTorque/analog:i");
    }

}

bool ContactSafetyThread::resetBaseline()
{

    Bottle* forceTorqueData = _forceTorque_in.read(true);
    if(!forceTorqueData->isNull())
    {
        _baseLine = sqrt(
                    pow(forceTorqueData->get(0).asDouble(), 2) +
                    pow(forceTorqueData->get(1).asDouble(), 2) +
                    pow(forceTorqueData->get(2).asDouble(), 2));
    }
    else{
        return false;
    }
    _collisionDetected = false;
}

bool ContactSafetyThread::collisionDetected()
{
    return _collisionDetected;
}


void ContactSafetyThread::setForceThreshold(double desiredForceThreshold)
{
    _forceThreshold = desiredForceThreshold;
}



bool ContactSafetyThread::threadInit(){

    if(_robotHand == NULL){
        cout << _dbgtag << " failed to initialise -- robot hand points to NULL, aborting" << endl;
        return false;
    }

    // Reset the force baseline
    if(!resetBaseline()){
        cerr << _dbgtag << "failed to initialise, force baseline could not be calculated." << endl;
        return false;
    }

    return true;
}


} // namespace objectExploration
