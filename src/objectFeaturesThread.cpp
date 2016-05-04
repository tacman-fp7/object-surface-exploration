#include <objectFeaturesThread.h>
#include <yarp/os/Network.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <vector>
#include <iCub/iKin/iKinFwd.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <new>
#include <math.h>
#include <yarp/os/Time.h>



namespace objectExploration
{

using yarp::os::Network;
using yarp::os::Value;
using yarp::os::Bottle;
using std::cout;
using std::endl;
using std::cerr;
using yarp::os::Mutex;



void ObjectFeaturesThread::run()
{

    // Read the fingertip position
    Vector fingertipPosition;

    //getIndexFingertipPosition(fingertipPosition);
    publishFingertipPosition(fingertipPosition);

    publishContactState(_contactState);
}



/////////// Accessor and mutators ///////////



/*void ObjectFeaturesThread::setEndPose ( Vector& pos, Vector& orient )
{
    if(pos[0] > _robotReachableSpace.disasterX)
    {
        cerr << _dbgtag << "Invalid x position" << endl;
        return;
    }
    _desiredEndPosition = pos;
    _desiredEndOrientation = orient;
    _desiredEndPose_isValid = true;
    updateRobotReachableSpace();
    printPose(pos, orient);
}
*/

/*void ObjectFeaturesThread::setStartingPose ( Vector& pos, Vector& orient )
{
    if(pos[0] > _robotReachableSpace.disasterX)
    {
        cerr << _dbgtag << "Invalid x position" << endl;
        return;
    }
    //cout << "Starting pose set" << endl;
    _desiredStartingPosition = pos;
    _desiredStartingOrientation = orient;
    _desiredStartingPose_isValid = true;
    updateRobotReachableSpace(); // This must be done after isValid is set to true;
    printPose(pos, orient);

}*/

/*bool ObjectFeaturesThread::getDesiredEndPose ( Vector& pos, Vector& orient )
{
    if(_desiredEndPose_isValid)
    {
        pos = _desiredEndPosition;
        orient = _desiredEndOrientation;
    }
    return _desiredEndPose_isValid;
}*/



/*bool ObjectFeaturesThread::setWayPointGP(yarp::sig::Vector pos, yarp::sig::Vector orient)
{
    // Breaching this will be disasterous for the robot!
    if(pos[0] >= _robotReachableSpace.disasterX)
    {
        cerr << _dbgtag << "Cannot have positive x-axis value" << endl;
        _wayPoint_isValid = false;
        return false;
    }


    pos[2] = _desiredStartingPosition[2];
    setWayPoint (pos, orient );
    _wayPoint_isValid = true;

    return true;
}*/

/*bool ObjectFeaturesThread::setWayPoint ( Vector pos, Vector orient )
{
    _wayPoint_isValid = true;

    // Breaching this will be disasterous for the robot!
    if(pos[0] >= _robotReachableSpace.disasterX)
    {
        cerr << _dbgtag << "Cannot have positive x-axis value" << endl;
        _wayPoint_isValid = false;
        return false;
    }

    if(pos[0] < _robotReachableSpace.minX )
    {
        cerr << _dbgtag << "X position outside allowable range ( "
             << _robotReachableSpace.minX << ", "
             <<  _robotReachableSpace.maxX << "): " << pos[0] << endl;

        pos[0] = _robotReachableSpace.minX;
        _wayPoint_isValid =  false;
        //return _wayPoint_isValid;
    }

    if( pos[0] > _robotReachableSpace.maxX)
    {
        cerr << _dbgtag << "X position outside allowable range ( "
             << _robotReachableSpace.minX << ", "
             <<  _robotReachableSpace.maxX << "): " << pos[0] << endl;

        pos[0]  = _robotReachableSpace.maxX;
        _wayPoint_isValid = false;
        //return _wayPoint_isValid;
    }


    if(pos[1] < _robotReachableSpace.minY )
    {
        cerr << _dbgtag << "Y position outside allowable range ( "
             << _robotReachableSpace.minY << ", "
             <<  _robotReachableSpace.maxY << "): " << pos[1] << endl;

        pos[1] = _robotReachableSpace.minY;
        _wayPoint_isValid = false;

    }

    if( pos[1] > _robotReachableSpace.maxY)
    {
        cerr << _dbgtag << "Y position outside allowable range ( "
             << _robotReachableSpace.minY << ", "
             <<  _robotReachableSpace.maxY << "): " << pos[1] << endl;

        pos[1] = _robotReachableSpace.maxY;
        _wayPoint_isValid = false;
        //return _wayPoint_isValid;
    }

    if(pos[2] < _robotReachableSpace.minZ)
    {

        cerr << _dbgtag << "Exceeded the z-axis limit ( " << _robotReachableSpace.minZ << " ): " << pos[2] << endl;
        pos[2] = _robotReachableSpace.minZ;

        if(_wayPointPos[2] == _robotReachableSpace.minZ)
            _wayPoint_isValid = false;
    }

    if(pos[2] > _robotReachableSpace.maxZ)
    {
        cerr << _dbgtag << "Exceeded the z-axis limit ( " << _robotReachableSpace.maxZ << " ): " << pos[2] << endl;
        pos[2] = _robotReachableSpace.maxZ;

        if(_wayPointPos[2] == _robotReachableSpace.maxZ)
            _wayPoint_isValid = false;
    }


    _wayPointPos = pos;
    _wayPointOrient = orient;

    return _wayPoint_isValid;

}
*/

/*bool ObjectFeaturesThread::getWayPoint ( Vector& pos, Vector& orient, bool invalidateWayPoint )
{

    bool ret = _wayPoint_isValid;
    //if(_wayPoint_isValid)
    //{
    pos = _wayPointPos;
    orient = _wayPointOrient;
    _wayPoint_isValid = !invalidateWayPoint;
    //  return true;
    //}
    return ret;
}*/

/*bool ObjectFeaturesThread::getStartingPose ( Vector& pos, Vector& orient )
{
    if(_desiredStartingPose_isValid)
    {
        pos = _desiredStartingPosition;
        orient = _desiredStartingOrientation;
    }
    return _desiredStartingPose_isValid;

}*/

/*bool ObjectFeaturesThread::getEndingPose( Vector& pos, Vector& orient )
{
    if(_desiredEndPose_isValid)
    {
        pos = _desiredEndPosition;
        orient = _desiredEndOrientation;
    }
    return _desiredEndPose_isValid;

}*/
/*void ObjectFeaturesThread::printPose ( Vector& pos, Vector& orient )
{
    cout << "Position: " << pos.toString() << endl;
    cout << "Orientation: " << orient.toString() << endl;
}*/

/*double ObjectFeaturesThread::getContactForce()
{
    _tactileMutex.lock();
    double temp = _contactForce;
    _tactileMutex.unlock();

    return temp;
}*/




bool ObjectFeaturesThread::threadInit()
{
    yarp::os::RateThread::threadInit();

    bool ret = true;

    _contactState = 0;

    _dbgtag = "\n\nObjectFeaturesThread.cpp: ";   


    _contactStatePort_out.open("/object-exploration/contact/state:o");
    _fingertipPosition_out.open("/object-exploration/fingertip/position:o");
    _fingertipControlPort_out.open("/object-exploration/fingertip/control:o");
    ///////////////////////////////



    if(ret)
        cout << "Object features thread configured" << endl;
    else
        cerr << _dbgtag << "Error, object features thread failed during configuration" << endl;

     return ret;
}

void ObjectFeaturesThread::threadRelease()
{
   // delete _objectSurfaceModelGP;

}



void ObjectFeaturesThread::publishFingertipControl(Bottle controlCommand)
{
    if(controlCommand.size() < 3)
        cout << "Fingertip control command needs more data" << endl;
    Bottle &data = _fingertipControlPort_out.prepare();
    data.clear();
    data = controlCommand;
    _fingertipControlPort_out.writeStrict();
}

void ObjectFeaturesThread::publishFingertipPosition(Vector pos)
{
    Bottle &data = _fingertipPosition_out.prepare();
    data.clear();
    data.addDouble(pos[0]);
    data.addDouble(pos[1]);
    data.addDouble(pos[2]);
    _fingertipPosition_out.writeStrict();

}

void ObjectFeaturesThread::publishContactState(int contactState)
{
    Bottle &data = _contactStatePort_out.prepare();
    data.clear();
    data.addInt(contactState);
    _contactStatePort_out.writeStrict();
}

ObjectFeaturesThread::~ObjectFeaturesThread()
{
    //_contactForceCoPPort.close();
    //_armPositionPort.close();

    _contactStatePort_out.close();
    _fingertipPosition_out.close();
}

/*void ObjectFeaturesThread::setArmController_cart(yarp::dev::ICartesianControl *cartesianCtrl)
{
    _armCartesianCtrl = cartesianCtrl;
}*/

/*void ObjectFeaturesThread::setArmController_jnt(yarp::dev::IEncoders *encoder, yarp::dev::IPositionControl *jointCtrl)
{

    _armEncoder = encoder;
    _armJointPositionCtrl = jointCtrl;
}*/

/*void ObjectFeaturesThread::setArmController_mode(yarp::dev::IControlMode2 *armJointCtrlmode)
{
    _armJointModeCtrl = armJointCtrlmode;
}*/

ObjectFeaturesThread::ObjectFeaturesThread ( int period, ResourceFinder rf ) : RateThread ( period )
{

    //_explorationThreadPeriod = 20;








    _moduleName = rf.check("moduleName", Value("object-exploration-server"),
                            "module name (string)").asString().c_str();


}



/*void ObjectFeaturesThread::updateRobotReachableSpace()
{
    if(_desiredStartingPose_isValid) // && _desiredEndPose_isValid)
    {
        _robotReachableSpace.minX = _desiredStartingPosition[0] - 0.30; //Maximum width 13 cm + 2 cm leeway
        _robotReachableSpace.maxX = _desiredStartingPosition[0] + 0.10;
        _robotReachableSpace.minZ = _desiredStartingPosition[2] - 0.05;
        _robotReachableSpace.maxZ = _desiredStartingPosition[2] + 0.05;

    }

    if(_desiredStartingPose_isValid && _desiredEndPose_isValid)
    {
        if(_desiredStartingPosition[1] < _desiredEndPosition[1])
        {
            _robotReachableSpace.minY = _desiredStartingPosition[1] - 0.08;
            _robotReachableSpace.maxY = _desiredEndPosition[1] + 0.08;

        }
        else
        {
            _robotReachableSpace.minY = _desiredEndPosition[1] - 0.08;
            _robotReachableSpace.maxY = _desiredStartingPosition[1] + 0.08;

        }
    }
}*/

/*const string& ObjectFeaturesThread::getArm()
{
    return _arm;
}*/



/*const string& ObjectFeaturesThread::getControllerType()
{
    return _controller;
}*/

/*const string& ObjectFeaturesThread::getRobotName()
{
    return _robotName;
}*/

/*const int& ObjectFeaturesThread::getTrajectoryTime()
{
    return _trajectoryTime;
}*/

/*const int& ObjectFeaturesThread::getExplorationThreadPeriod()
{
    return _explorationThreadPeriod;
}*/



/*const double& ObjectFeaturesThread::getDesiredForce()
{
    return _desiredFroce;
}*/

} // namespace objectExploration
