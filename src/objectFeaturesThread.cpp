#include <objectFeaturesThread.h>
#include <yarp/os/Network.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <vector>


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
    Bottle* contactForceCoP = _contactForceCoPPort.read(true); // Wait for data

    //// Read the corresponding arm position. //////
    //Bottle* armPose = _armPositionPort.read(true);

    if(contactForceCoP == NULL )
    {
        // TODO: figure out why it gets run when deleting the thread  <--- becuase it is was waiting for the tactile data
        cerr << _dbgtag << "Run: Null pointers!" << endl;
        return;
    }
    if(contactForceCoP->isNull())
    {
        cerr << "Did not receive tactile or arm data" << endl;
        return;
    }

    double encoderValue;
    _armPoseMutex.lock();
    _armCartesianCtrl->getPose(_armPosition, _armOrientation);
    /*for (int i = 0; i < 3; i++)
        _armPosition[i] = armPose->get(i).asDouble();
    for (int i = 3; i < 7; i++)
        _armOrientation[i-3] = armPose->get(i).asDouble();*/

    _armJointCtrl->getEncoder(_proximalJoint_index, &encoderValue);

    _armPoseMutex.unlock();

    cout << _armPosition.toString() << " : " << _proximalJoint_index << " : " << encoderValue << endl;

    //cout << armPose->toString() << endl << endl;
    //cout << _armPosition.toString() << endl;
    //cout << _armOrientation.toString() << endl << endl;

    _tactileMutex.lock();
    _contactForce = contactForceCoP->get(0).asDouble(); // Contact force field is the first one
    _tactileMutex.unlock();


}


/// Urgh

void ObjectFeaturesThread::writeToFingerController(std::string command)
{
    //Bottle rpcCommand, rpcResponse;
    //rpcCommand.addString(command);

    Bottle &message = _fingerController_port.prepare();
    message.clear();

    message.clear();

    //// Copied from Massimo
    if (!command.empty()){

        char *commandChar = new char[command.length() + 1];
        strcpy(commandChar,command.c_str());
        std::vector<string> wordList;
        char *target;

        target = strtok(commandChar," ");
        while(target != NULL){
            wordList.push_back(target);
            target = strtok(NULL," ");
        }

        for(size_t i = 0; i < wordList.size(); i++){
            message.add(yarp::os::Value(wordList[i]));
        }

    }


    _fingerController_port.writeStrict();

}

/////////// Accessor and mutators ///////////

Vector ObjectFeaturesThread::getPosition()
{ 
    _armPoseMutex.lock();
    Vector temp = _armPosition;
    _armPoseMutex.unlock();
    return temp;

}

bool ObjectFeaturesThread::getHomePose ( Vector& pos, Vector& orient )
{
    if(_homePose_isValid)
    {
        pos = _homePosition;
        orient = _homeOrientation;
    }
    else
        cerr << "Home pose is invalid" << endl;

    return _homePose_isValid;
}

void ObjectFeaturesThread::setHomePose ( Vector& pos, Vector& orient )
{
    _homePosition = pos;
    _homeOrientation = orient;
    _homePose_isValid = true;
    printPose(pos, orient);
}


void ObjectFeaturesThread::setEndPose ( Vector& pos, Vector& orient )
{
    _desiredEndPosition = pos;
    _desiredEndOrientation = orient;
    _desiredEndPose_isValid = true;
    printPose(pos, orient);
}

void ObjectFeaturesThread::setStartingPose ( Vector& pos, Vector& orient )
{
    _desiredStartingPosition = pos;
    _desiredStartingOrientation = orient;
    _desiredStartingPose_isValid = true;
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

void ObjectFeaturesThread::setWayPoint ( Vector pos, Vector orient )
{
    _wayPointPos = pos;
    _wayPointOrient = orient;
    _wayPoint_isValid = true;
    //printPose(pos, orient);
}


bool ObjectFeaturesThread::getWayPoint ( Vector& pos, Vector& orient, bool invalidateWayPoint )
{
    if(_wayPoint_isValid)
    {
        pos = _wayPointPos;
        orient = _wayPointOrient;
        _wayPoint_isValid = !invalidateWayPoint;
        return true;
    }
    return false;
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

void ObjectFeaturesThread::printPose ( Vector& pos, Vector& orient )
{
    cout << "Position: " << pos.toString() << endl;
    cout << "Orientation: " << orient.toString() << endl;
}

double ObjectFeaturesThread::getForce()
{
    _tactileMutex.lock();
    double temp = _contactForce;
    _tactileMutex.unlock();

    return temp;
}


Vector ObjectFeaturesThread::getOrientation()
{
    _armPoseMutex.lock();
    Vector temp = _armOrientation;
    _armPoseMutex.unlock();
    return temp;
}



bool ObjectFeaturesThread::threadInit()
{
    yarp::os::RateThread::threadInit();

    bool ret = true;

  _dbgtag = "objectFeaturesThread.cpp: ";

    /*   /////////////////// Connect to the tactile sensor port /////////////////
    if(!_tactilePort.open("/objectExploration/tactileSensors/" + _arm + "_hand")){
        ret = false;
        printf("Failed to open local tactile port\n");
    }

    Network::connect("/" + _robotName + "/skin/" + _arm + "_hand_comp",
                     "/objectExploration/tactileSensors/" + _arm + "_hand");
*/
    ////////////////// Connect to the forceCoP port ///////////////////////
    if(!_contactForceCoPPort.open("/" + _moduleName + "/skin/" + _arm + "_hand/" + _whichFinger + "/force-CoP:i"))
    {
        ret = false;
        cerr << _dbgtag << "Failed to open " << "/" << _moduleName << "/skin" << _arm << "_hand/" <<
                _whichFinger << "/force-CoP:i" << endl;
//        printf("Failed to open local tactile port\n");
    }

    ///////////////////////////////////////
    //TODO: Change the incoming port!
    ////////////////////////////////////////
    Network::connect("/force-reconstruction/right_index/force-CoP",
                     "/" + _moduleName + "/skin/" + _arm + "_hand/" + _whichFinger + "/force-CoP:i");

    /// Connect to the finger controller RPC port

    if(!_fingerController_port.open("/" + _moduleName + "/" + _arm + "_hand/" + _whichFinger + "/command:o"))
    {
        ret = false;
        cerr << _dbgtag << "Failed to open local fingerController port" << endl;
    }

    Network::connect("/" + _moduleName + "/" + _arm + "_hand/" + _whichFinger + "/command:o",
                     _fingerControllerPortName);



    /////////////// Opening arm pose port and connecting to it //////////////
  /*  if(!_armPositionPort.open("/" + _moduleName + "/" + _controllerName + "/" + _arm + "_arm/pose:i"))
    {
        ret = false;
        cout << "Failed to open local arm pose port" << endl;
    }
    //icubSim/cartesianController/left_arm/state:o
    if(!Network::connect("/" + _robotName  + "/" + _controllerName + "/" + _arm + "_arm/state:o",
                         "/" + _moduleName + "/" + _controllerName + "/" + _arm + "_arm/pose:i"))
    {
        ret = false;
        cerr << _dbgtag << "Failed to connect to the arm pose port" << endl;
    }
*/
    ///////////////////////////////////////////////////////////////////////////////
    ///// Open the joint port and connect to it ///////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    /*if(!_armJointPort_in.open("/" + _moduleName + "/" + _arm + "_arm/state:i" ))
    {
        ret = false;
        cerr << _dbgtag << "Failed to open local arm state port" << endl;
    }


    if(!Network::connect("/" + _robotName + "/" + _arm + "_arm/state:o", "/" + _moduleName + "/" + _arm + "_arm/state:i"))
    {
        ret = false;
        cerr << _dbgtag << "Failed to connect to the arm state port" << endl;
    }*/

    // TODO: figure out why removing this crashes the application
    // is it because the the network connection needs time?
    if(ret)
        cout << "Object features thread configured" << endl;
    else
        cerr << _dbgtag << "Error, object features thread failed during configuration" << endl;

    return ret;
}

void ObjectFeaturesThread::threadRelease()
{

}

ObjectFeaturesThread::~ObjectFeaturesThread()
{
    _contactForceCoPPort.close();
    //_armPositionPort.close();

}

void ObjectFeaturesThread::setArmController_cart(yarp::dev::ICartesianControl *cartesianCtrl)
{
    _armCartesianCtrl = cartesianCtrl;
}

void ObjectFeaturesThread::setArmController_jnt(yarp::dev::IEncoders *jointCtrl)
{
    _armJointCtrl = jointCtrl;
}

ObjectFeaturesThread::ObjectFeaturesThread ( int period, ResourceFinder rf ) : RateThread ( period )
{

    // Some sane and safe default values
    _trajectoryTime = 5; // By default take 5 seconds to complete a trajectory
    _maintainContactPeriod = 20;
    _readTactilePeriod = 20;
    _explorationThreadPeriod = 20;

    _desiredFroce = 0;

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


    _armOrientation.resize(4);
    _armPosition.resize(3);


    _contactForce = 0;
    _rf = rf;

    _armCartesianCtrl = NULL;
    _armJointCtrl = NULL;

    _proximalJointAngle = 0;
    _proximalJoint_index = 0;

    ////////////// read the parameters from the config file ///////////////
    this->readParameters();
}

bool ObjectFeaturesThread::readParameters()
{

    _moduleName = _rf.check("moduleName", Value("object-exploration-server"),
                                 "module name (string)").asString().c_str();


    Bottle &robotParameters = _rf.findGroup("RobotParameters");
    if(!robotParameters.isNull()){
        // Read the arm configuration
        _arm = robotParameters.check("arm", Value("left")).asString();
        _robotName = robotParameters.check("robotName", Value("icubSim")).asString();
        _controller = robotParameters.check("controller", Value("Error")).asString();
        _controllerName = robotParameters.check("controllerName", Value("Error")).asString();
        _trajectoryTime = robotParameters.check("_trajectoryTime", Value(5)).asInt();
        _fingerControllerPortName = robotParameters.check("fingerControllerPortName",
                                                          Value("/plantIdentification/cmd:i")).asString();
        _whichFinger = robotParameters.check("whichFinger", Value("left_index")).asString();

    }


    Bottle& explorationParameters = _rf.findGroup("ExplorationParameters");
    Bottle* startingPose;
    Bottle* endPose;
    if(!explorationParameters.isNull())
    {
        _maintainContactPeriod = explorationParameters.check("maintainContactPeriod", Value(20)).asInt();
        _desiredFroce = explorationParameters.check("desiredFroce", Value(0)).asDouble();
        _readTactilePeriod = explorationParameters.check("readTactilePeriod", Value(20)).asInt();
        _explorationThreadPeriod = explorationParameters.check("explorationThreadPeriod", Value(20)).asInt();
        startingPose = explorationParameters.find("startingPose").asList();
        endPose = explorationParameters.find("endPose").asList();
    }


    //////// Initialise the starting pose //////
    if(startingPose->size() < 7)
        cout << "startingPose is invalid" << endl;
    else
    {
        for(int i = 0; i < 3; i++)
            _desiredStartingPosition[i] = startingPose->get(i).asDouble();
        for(int i = 3; i < 7; i++)
            _desiredStartingOrientation[i-3] = startingPose->get(i).asDouble();
        _desiredStartingPose_isValid = true;
    }

    if(endPose->size() < 7)
        cout << "End pose is invalid!" << endl;
    else
    {
        for(int i = 0; i < 3; i++)
            _desiredEndPosition[i] = endPose->get(i).asDouble();
        for(int i = 3; i < 7; i++)
            _desiredEndOrientation[i-3] = endPose->get(i).asDouble();
        _desiredEndPose_isValid = true;
    }

    cout << "Read the following configuration from the file:" << endl;
    cout << "Robot name: " << _robotName << endl;
    cout << "Arm: " << _arm << endl;
    cout << "Controller: " << _controller << endl;
    cout << "Controller name: " << _controllerName << endl;
    cout << "Trajectory time: " << _trajectoryTime << endl;

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

    cout << "Maintain contact thread period: " << _maintainContactPeriod << endl;
    cout << "Desired force: " << _desiredFroce << endl;
    cout << "Read tactile sensors thread period: " << _readTactilePeriod << endl;
    cout << "Exploration thread period: " << _explorationThreadPeriod << endl;
    cout << endl;

    if(_whichFinger.compare(_whichFinger.size()-5, 5, "index")==0)
        _proximalJoint_index = 11;
    else
        _proximalJoint_index = 9;

}

const string& ObjectFeaturesThread::getArm()
{
    return _arm;
}

const string& ObjectFeaturesThread::getControllerName()
{
    return _controllerName;
}

const string& ObjectFeaturesThread::getControllerType()
{
    return _controller;
}

const string& ObjectFeaturesThread::getRobotName()
{
    return _robotName;
}

const int& ObjectFeaturesThread::getTrajectoryTime()
{
    return _trajectoryTime;
}

const int& ObjectFeaturesThread::getExplorationThreadPeriod()
{
    return _explorationThreadPeriod;
}

const int& ObjectFeaturesThread::getMaintainContactPeriod()
{
    return _maintainContactPeriod;
}

const double& ObjectFeaturesThread::getDesiredForce()
{
    return _desiredFroce;
}

} // namespace objectExploration
