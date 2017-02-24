#include <contactSafetyThread.h>
#include <stdio.h>
#include <iostream>
#include <yarp/sig/Vector.h>
#include <yarp/os/Network.h>
#include <math.h>
#include <yarp/os/ResourceFinder.h>
#include <sstream>

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
            if(!_collisionDetected){
                cout << _dbgtag << "exceeded contact force: " << fabs(resultant - _baseLine) << endl;
            }
            _robotHand->stopControl();
            _collisionDetected = true;

        }
        else{
            _collisionDetected = false;
        }
    }
}


double ContactSafetyThread::getForceThreshold(){
    double ret;
    _forceThreshold_mutex.lock();
    ret = _forceThreshold;
    _forceThreshold_mutex.unlock();
}

ContactSafetyThread::ContactSafetyThread(int period,  Hand *robotHand)
    :RateThread(period), _robotHand(robotHand){


    _dbgtag = "Contact safety thread: ";
    _collisionDetected = false;
    _forceThreshold = 3; // initialise to a meaningful value
    // Set the force threshold from the config file
    setForceThreshold(_robotHand->getContactSafetyForceThereshold());

    /// Force torque//////

    string localPortName = "/safeConact/" + _robotHand->getArmName() + "_arm/forceTorque/analog:i";
    string remotePortName =  "/" + _robotHand->getRobotName() + "/" + _robotHand->getArmName() + "_arm/analog:o";



    if(!_forceTorque_in.open(localPortName)){
        cerr << _dbgtag << "could not open local port: " << localPortName << endl;
        throw std::runtime_error(_dbgtag + "could not open local port: " + localPortName + "\n");
    }

    if(!yarp::os::Network::connect(remotePortName, localPortName))
    {
        throw std::runtime_error(_dbgtag + "could not connect to remote port: " + remotePortName + "\n");
    }





}


bool ContactSafetyThread::resetBaseline(){

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

    return true;
}

bool ContactSafetyThread::collisionDetected()
{
    return _collisionDetected;
}


void ContactSafetyThread::setForceThreshold(double desiredForceThreshold)
{
    _forceThreshold_mutex.lock();
    _forceThreshold = desiredForceThreshold;
    _forceThreshold_mutex.unlock();
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
