#include <maintainContactThread.h>
#include <stdio.h>
#include <iostream>
#include <yarp/sig/Vector.h>
#include <math.h>

using std::cerr;
using std::cout;
using std::endl;
using yarp::sig::Vector;

namespace objectExploration
{


bool MaintainContactThread::setDesiredForce(double desiredForce)
{
    _desiredForce = desiredForce;
}

void MaintainContactThread::run()
{

    // Read the tactile data
    double force = _objectFeatures->getContactForce();

    // Read the corresponding robot positon
    //TODO: Gotta make sure there is deep copy constructor
    Vector px, po; // position and getOrientation
    px = _objectFeatures->getPosition();
    po = _objectFeatures->getOrientation();


    //cout << _objectFeatures->getForce() << endl;
    // Read the current position
    //cout << _objectFeatures->getPosition().toString() << endl;
    //cout << _objectFeatures->getOrientation().toString() << endl;

    // Calculate the action to be taken

    // Introduce a new waypoint
    // Change the z pos


    //px[2] += 0.1;





    //static double st = 0;

    Vector pxd, pod;
    _objectFeatures->getDesiredEndPose(pxd, pod);
    //pxd[2] =  px[2] + sin(st)/5;
    //st += 0.1;
    cout << "Force: " << force << endl;
    pxd[2] = px[2] + _positionPID.update(force);

    _objectFeatures->setWayPoint(pxd, pod);


}


bool MaintainContactThread::threadInit()
{
    yarp::os::RateThread::threadInit();

    if(_objectFeatures == NULL){
        cout << "MaintainContactThread failed: objectFeatures points to NULL, aborting" << endl;
        return false;
    }
    else
    {
        cout << "MaintainContactThread configured" << endl;
    }

    pidParams_t PIDParams;

    memset(&PIDParams, 0, sizeof(pidParams_t));
    PIDParams.Kp = -0.1;
    PIDParams.outMax = 0.1;
    PIDParams.outMin = -0.1;

    _positionPID.setParameters(PIDParams);
    _positionPID.setSetpoint(0.5);

    return true;
}


void MaintainContactThread::threadRelease()
{
    yarp::os::RateThread::threadRelease();
}

} // namespace objectExploration
