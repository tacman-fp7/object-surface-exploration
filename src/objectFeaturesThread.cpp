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


/*
 * 192 for hand data, where 1-60 are taxels of fingertips (12 each in this order:
 * index, middle, ring, little, thumb); 61-96 zeros; 97-144 palm taxels
 * (inside these, 108, 120, 132, and 140 are thermal pads ~ 0s); 145-192 zeros.
 */
void ObjectFeaturesThread::run()
{


    //cout << _dbgtag << "Still running!" << endl;
    ///// Read the tactile data ///////
    //Bottle* tactileData = _tactilePort.read(true); // Wait for data
    //Bottle* contactForceCoP = _contactForceCoPPort.read(true); // Wait for data

    //// Read the corresponding arm position. //////
    //Bottle* armPose = _armPositionPort.read(true);

    /*if(contactForceCoP == NULL )
    {
        // TODO: figure out why it gets run when deleting the thread  <--- becuase it is was waiting for the tactile data
        cerr << _dbgtag << "Run: Null pointers!" << endl;
        return;
    }
    if(contactForceCoP->isNull())
    {
        cerr << "Did not receive tactile or arm data" << endl;
        return;
    }*/


    //_tactileMutex.lock();
    //_contactForce = contactForceCoP->get(0).asDouble(); // Contact force field is the first one
    //_tactileMutex.unlock();


    // Read the fingertip position
    Vector fingertipPosition;

    //getIndexFingertipPosition(fingertipPosition);
    publishFingertipPosition(fingertipPosition);

    publishContactState(_contactState);
}



/////////// Accessor and mutators ///////////



void ObjectFeaturesThread::setEndPose ( Vector& pos, Vector& orient )
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

void ObjectFeaturesThread::setStartingPose ( Vector& pos, Vector& orient )
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

}

bool ObjectFeaturesThread::getDesiredEndPose ( Vector& pos, Vector& orient )
{
    if(_desiredEndPose_isValid)
    {
        pos = _desiredEndPosition;
        orient = _desiredEndOrientation;
    }
    return _desiredEndPose_isValid;
}



bool ObjectFeaturesThread::setWayPointGP(yarp::sig::Vector pos, yarp::sig::Vector orient)
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
}

bool ObjectFeaturesThread::setWayPoint ( Vector pos, Vector orient )
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


bool ObjectFeaturesThread::getWayPoint ( Vector& pos, Vector& orient, bool invalidateWayPoint )
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
}

bool ObjectFeaturesThread::getStartingPose ( Vector& pos, Vector& orient )
{
    if(_desiredStartingPose_isValid)
    {
        pos = _desiredStartingPosition;
        orient = _desiredStartingOrientation;
    }
    return _desiredStartingPose_isValid;

}

bool ObjectFeaturesThread::getEndingPose( Vector& pos, Vector& orient )
{
    if(_desiredEndPose_isValid)
    {
        pos = _desiredEndPosition;
        orient = _desiredEndOrientation;
    }
    return _desiredEndPose_isValid;

}
void ObjectFeaturesThread::printPose ( Vector& pos, Vector& orient )
{
    cout << "Position: " << pos.toString() << endl;
    cout << "Orientation: " << orient.toString() << endl;
}

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


    ///////// Connect to the force port ///////////////////////

//  /force-cop-estimator/left_index/force:o


    ////////////////// Connect to the forceCoP port ///////////////////////
   /* if(!_contactForceCoPPort.open("/" + _moduleName + "/skin/" + _arm + "_hand/" + _whichFinger + "/force-CoP:i"))
    {
        ret = false;
        cerr << _dbgtag << "Failed to open " << "/" << _moduleName << "/skin" << _arm << "_hand/" <<
                _whichFinger << "/force-CoP:i" << endl;
        _isExplorationValid = false;
        //        printf("Failed to open local tactile port\n");
    }*/

    ///////////////////////////////////////
    //TODO: Change the incoming port!
    ////////////////////////////////////////
  /*  if(!Network::connect("/force-reconstruction/" + _arm + "_index/force-CoP",
                         "/" + _moduleName + "/skin/" + _arm + "_hand/" + _whichFinger + "/force-CoP:i"))
    {
        _isExplorationValid = false;
        cerr << _dbgtag << "Failed to connect:" << endl;
        cerr << "/force-reconstruction/" + _arm + "_index/force-CoP" << " and" << endl;
        cerr << "/" + _moduleName + "/skin/" + _arm + "_hand/" + _whichFinger + "/force-CoP:i" << endl << endl;
    }*/







    _contactStatePort_out.open("/object-exploration/contact/state:o");
    _fingertipPosition_out.open("/object-exploration/fingertip/position:o");
    _fingertipControlPort_out.open("/object-exploration/fingertip/control:o");
    ///////////////////////////////



    if(ret)
        cout << "Object features thread configured" << endl;
    else
        cerr << _dbgtag << "Error, object features thread failed during configuration" << endl;

    _objectSurfaceModelGP = new objectExploration::SurfaceModelGP("hut"); //TODO: use the config file
    return ret;
}

void ObjectFeaturesThread::threadRelease()
{
    delete _objectSurfaceModelGP;

}

bool ObjectFeaturesThread::prepGP()
{

    bool ret = true;

    _objectSurfaceModelGP->loadContactData("blindSearch"); //TODO: put this in a config file
    _objectSurfaceModelGP->trainModel();
    _objectSurfaceModelGP->updateSurfaceEstimate();
    //_objectSurfaceModelGP->saveMeshCSV();


    Vector maxVariancePos;
    Vector orient;

    // Get the valid point from object features
    if(getWayPoint(maxVariancePos, orient))
    {

        // I have to make sure the new waypoint is valid
        if(_objectSurfaceModelGP->getMaxVariancePose(maxVariancePos))
            ret = ret && setWayPoint(maxVariancePos, orient);

    }
    else
    {
        cerr << _dbgtag << "Invalid initial waypoint" << endl;
        ret = false;
    }

    return ret;

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

    // Some sane and safe default values
    //_trajectoryTime = 5; // By default take 5 seconds to complete a trajectory
    //_maintainContactPeriod = 20;
    //_readTactilePeriod = 20;
    _explorationThreadPeriod = 20;
    _isExplorationValid = true; // assume true,

    //_desiredFroce = 0;

    _desiredStartingPose_isValid = false;
    _desiredStartingPosition.resize(3); // x,y,z position
    _desiredStartingOrientation.resize(4); // Axis angle

    _desiredEndPose_isValid = false;
    _desiredEndOrientation.resize(4);
    _desiredEndPosition.resize(3);

    _homePose_isValid = false;
    _homeOrientation.resize(4);
    _homePosition.resize(3);

    _wayPoint_isValid = false;
    _wayPointOrient.resize(4);
    _wayPointPos.resize(3);


    //_armOrientation.resize(4);
    //_armPosition.resize(3);


    //_contactForce = 0;
    _rf = rf;




    ////////////// read the parameters from the config file ///////////////
    this->readParameters();




}

bool ObjectFeaturesThread::readParameters()
{

    ///// Set a safe workspace for the robot
    //// This get updated when I read the
    /// Starting and ending positions

    _robotReachableSpace.minX = -0.4;
    _robotReachableSpace.maxX = -0.2;
    _robotReachableSpace.minY =  0; // This works for both arms
    _robotReachableSpace.maxY =  0; // This works for both arms
    _robotReachableSpace.minZ = -0.1;
    _robotReachableSpace.maxZ =  0.1;
    _robotReachableSpace.disasterX = -0.2; // Beyond this point you will break the hand


    _moduleName = _rf.check("moduleName", Value("object-exploration-server"),
                            "module name (string)").asString().c_str();


    Bottle &robotParameters = _rf.findGroup("RobotParameters");
    if(!robotParameters.isNull()){
        // Read the arm configuration
        _arm = robotParameters.check("arm", Value("left")).asString();
        _robotName = robotParameters.check("robotName", Value("icubSim")).asString();
        _controller = robotParameters.check("controller", Value("Error")).asString();
        _controllerName = robotParameters.check("controllerName", Value("Error")).asString();
        //_trajectoryTime = robotParameters.check("trajectoryTime", Value(5)).asInt();

        _whichFinger = robotParameters.check("whichFinger", Value("left_index")).asString();

    }


    Bottle& explorationParameters = _rf.findGroup("ExplorationParameters");
    Bottle* startingPose;
    Bottle* endPose;
    if(!explorationParameters.isNull())
    {
        //_maintainContactPeriod = explorationParameters.check("maintainContactPeriod", Value(20)).asInt();
        //_desiredFroce = explorationParameters.check("desiredFroce", Value(0)).asDouble();
        //_readTactilePeriod = explorationParameters.check("readTactilePeriod", Value(20)).asInt();
        _explorationThreadPeriod = explorationParameters.check("explorationThreadPeriod", Value(20)).asInt();
        startingPose = explorationParameters.find("startingPose").asList();
        endPose = explorationParameters.find("endPose").asList();
    }


    //////// Initialise the starting pose //////
    if(startingPose->size() < 7)
        cout << "startingPose is invalid" << endl;
    else
    {
        Vector pos, orient;
        pos.resize(3);
        orient.resize(4);

        for(int i = 0; i < 3; i++)
            pos[i] = startingPose->get(i).asDouble();
        for(int i = 3; i < 7; i++)
            orient[i-3] = startingPose->get(i).asDouble();

        setStartingPose(pos, orient);

        _desiredStartingPose_isValid = true;
    }
    /////////////////////////// Naiwd fix this! ////////
    if(endPose->size() < 7)
        cout << "End pose is invalid!" << endl;
    else
    {
        for(int i = 0; i < 3; i++)
            _desiredEndPosition[i] = endPose->get(i).asDouble();
        for(int i = 3; i < 7; i++)
            _desiredEndOrientation[i-3] = endPose->get(i).asDouble();

        _desiredEndPose_isValid = true;
        updateRobotReachableSpace();
    }

    cout << "Read the following configuration from the file:" << endl;
    cout << "Robot name: " << _robotName << endl;
    cout << "Arm: " << _arm << endl;
    cout << "Controller: " << _controller << endl;
    cout << "Controller name: " << _controllerName << endl;
    //cout << "Trajectory time: " << _trajectoryTime << endl;

    if(_desiredStartingPose_isValid)
    {

        cout << "Starting position: " << _desiredStartingPosition.toString() << endl;
        cout << "Starting orientation: " << _desiredStartingOrientation.toString() << endl;
    }

    if(_desiredEndPose_isValid)
    {
        cout << "End position: " << _desiredEndPosition.toString() << endl;
        cout << "End orientation: " << _desiredEndOrientation.toString() << endl;
    }

   // cout << "Maintain contact thread period: " << _maintainContactPeriod << endl;
    //cout << "Desired force: " << _desiredFroce << endl;
    //cout << "Read tactile sensors thread period: " << _readTactilePeriod << endl;
    cout << "Exploration thread period: " << _explorationThreadPeriod << endl;
    cout << endl;

    /*if(_whichFinger.compare(_whichFinger.size()-5, 5, "index")==0)
        _proximalJoint_index = 11;
    else
        _proximalJoint_index = 9;
*/

}

void ObjectFeaturesThread::updateRobotReachableSpace()
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
}

const string& ObjectFeaturesThread::getArm()
{
    return _arm;
}



/*const string& ObjectFeaturesThread::getControllerType()
{
    return _controller;
}*/

const string& ObjectFeaturesThread::getRobotName()
{
    return _robotName;
}

/*const int& ObjectFeaturesThread::getTrajectoryTime()
{
    return _trajectoryTime;
}*/

const int& ObjectFeaturesThread::getExplorationThreadPeriod()
{
    return _explorationThreadPeriod;
}



/*const double& ObjectFeaturesThread::getDesiredForce()
{
    return _desiredFroce;
}*/

} // namespace objectExploration
