#include <objectFeaturesThread.h>
#include <yarp/os/Network.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <vector>
#include <iCub/iKin/iKinFwd.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <new>

#define M_PI   3.14159265358979323846264338328


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

    if(_armEncoder->getEncoder(_proximalJoint_index, &encoderValue))
        _proximalJointAngle = encoderValue;
    else
        cerr << _dbgtag << "Invalid proximal joint value." << endl;

    _armPoseMutex.unlock();

    //cout << _armPosition.toString() << " : " << _proximalJoint_index << " : " << encoderValue << endl;

    //cout << armPose->toString() << endl << endl;
    //cout << _armPosition.toString() << endl;
    //cout << _armOrientation.toString() << endl << endl;

    _tactileMutex.lock();
    _contactForce = contactForceCoP->get(0).asDouble(); // Contact force field is the first one
    _tactileMutex.unlock();


}

bool ObjectFeaturesThread::getArmPose(yarp::sig::Vector &pos, yarp::sig::Vector &orient)
{

    bool ret;
    _armPoseMutex.lock();
    ret = _armCartesianCtrl->getPose(_armPosition, _armOrientation);
    pos = _armPosition;
    orient = _armOrientation;
    _armPoseMutex.unlock();

    return ret;
}

bool ObjectFeaturesThread::prepHand()
{
    int numAxes;


    if(!_armEncoder->getAxes( &numAxes))
    {
        cerr << _dbgtag << "Could not read the number available arm axes." << endl;
        return false;
    }

    if(numAxes < 16)
    {
        cerr << _dbgtag << "Expected 16 axes, got" << numAxes << endl;
        return false;
    }



    /////_armJointPositionCtrl->setPositionMode();
    ///// Quick fixe /////

    /*if(!_armJointPositionCtrl->positionMove(7, 0)) //TODO: use the config file
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }


    if(!_armJointPositionCtrl->positionMove(8, 10)) //TODO: use the config file
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }*/

    for (int i=7; i < numAxes; i++)
    {
        if(!_armJointPositionCtrl->positionMove(i, 0))
        {
            cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
        }
    }

    if(!_armJointPositionCtrl->positionMove(12, 65)) //TODO: use the config file
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }

    if(!_armJointPositionCtrl->positionMove(13, 90)) //TODO: use the config file
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }

    if(!_armJointPositionCtrl->positionMove(14, 100)) //TODO: use the config file
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }

    if(!_armJointPositionCtrl->positionMove(15, 250)) //TODO: use the config file
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }

   return true;

}

bool ObjectFeaturesThread::openHand()
{
    int numAxes;


    if(!_armEncoder->getAxes( &numAxes))
    {
        cerr << _dbgtag << "Could not read the number available arm axes." << endl;
        return false;
    }

    if(numAxes < 16)
    {
        cerr << _dbgtag << "Expected 16 axes, got" << numAxes << endl;
        return false;
    }




    ////_armJointPositionCtrl->setPositionMode();
    ///// Quick fix /////
   /* if(!_armJointPositionCtrl->positionMove(7, 0)) //TODO: use the config file
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }


    if(!_armJointPositionCtrl->positionMove(8, 10)) //TODO: use the config file
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }*/

    for (int i=7; i < numAxes; i++)
    {
        if(!_armJointPositionCtrl->positionMove(i, 0))
        {
            cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
        }
    }


   return true;
}

void ObjectFeaturesThread::adjustIndexFinger()
{

    _armPoseMutex.lock();
    double tempProximalAngle = _proximalJointAngle;
    _armPoseMutex.unlock();

    if(!_armJointPositionCtrl->positionMove(12, 90 - tempProximalAngle)) //TODO: use the config file
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }
}

bool ObjectFeaturesThread::getFingertipPose(yarp::sig::Vector &pos, yarp::sig::Vector &orient)
{
    bool ret = true;


    int nEncs;


    _armEncoder->getAxes(&nEncs);
    Vector encs(nEncs);
    if(! (ret = _armEncoder->getEncoders(encs.data())))
    {
        cerr << _dbgtag << "Failed to read arm encoder data" << endl;
    }


//    cout << "Encoder data" << encs.toString() << endl;

    Vector joints;
    iCub::iKin::iCubFinger finger(_whichFinger);

    //cout << "Finger: " << _whichFinger << endl;


    finger.getChainJoints(encs, joints);

    cout << "Joints: " << joints.toString() << endl;

    //Convert the joints to radians.
    for (int j = 0; j < joints.size(); j++)
        joints[j] *= M_PI/180;

    yarp::sig::Matrix tipFrame = finger.getH(joints);

    Vector tip_x = tipFrame.getCol(3);
    Vector tip_o = yarp::math::dcm2axis(tipFrame);

    // I should have a mutex here specsific for the carteria view!
    _armPoseMutex.lock();

   if(!_armCartesianCtrl->attachTipFrame(tip_x, tip_o))
       ret = false;

    if(!_armCartesianCtrl->getPose(pos, orient))
    {
        cerr << _dbgtag << "Failed to read the fingertip pose" << endl;
        ret = false;
    }

    if(!_armCartesianCtrl->removeTipFrame())
        ret = false;

    _armPoseMutex.unlock();
    return ret;
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
    cout << "Starting pose set" << endl;
    _desiredStartingPosition = pos;
    _desiredStartingOrientation = orient;
    _zMax = pos[2] + 0.05;
    _zMin = pos[2] - 0.05;
    cout << "Max: " << _zMax << " Min: " << _zMin << endl;

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


    _wayPoint_isValid = true;

    if(pos[0] >= 0)
    {
        cerr << _dbgtag << "Cannot have positive x-axis value" << endl;
        _wayPoint_isValid = false;
        return;
    }



    // TODO: in a config file
    double min = _zMin;
    double max =  _zMax;

    //if(_wayPointPos[2] == max || _wayPointPos[2] == min)
    //{
    //    _wayPoint_isValid = false;
    //    return;

    //}

    if(pos[2] < min)
    {

        cerr << _dbgtag << "Exceeded the z-axis limit ( " << min << " ): " << pos[2] << endl;
        pos[2] = min;

        if(_wayPointPos[2] == min)
            _wayPoint_isValid = false;
    }

    if(pos[2] > max)
    {
        cerr << _dbgtag << "Exceeded the z-axis limit ( " << max << " ): " << pos[2] << endl;
        pos[2] = max;

        if(_wayPointPos[2] == max)
            _wayPoint_isValid = false;
    }


    _wayPointPos = pos;
    _wayPointOrient = orient;

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

void ObjectFeaturesThread::printPose ( Vector& pos, Vector& orient )
{
    cout << "Position: " << pos.toString() << endl;
    cout << "Orientation: " << orient.toString() << endl;
}

double ObjectFeaturesThread::getContactForce()
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

  _dbgtag = "\n\nObjectFeaturesThread.cpp: ";

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
        _isExplorationValid = false;
//        printf("Failed to open local tactile port\n");
    }

    ///////////////////////////////////////
    //TODO: Change the incoming port!
    ////////////////////////////////////////
    if(!Network::connect("/force-reconstruction/" + _arm + "_index/force-CoP",
                     "/" + _moduleName + "/skin/" + _arm + "_hand/" + _whichFinger + "/force-CoP:i"))
    {
        _isExplorationValid = false;
        cerr << _dbgtag << "Failed to connect:" << endl;
        cerr << "/force-reconstruction/" + _arm + "_index/force-CoP" << " and" << endl;
        cerr << "/" + _moduleName + "/skin/" + _arm + "_hand/" + _whichFinger + "/force-CoP:i" << endl << endl;
    }

    /// Connect to the finger controller RPC port

    if(!_fingerController_port.open("/" + _moduleName + "/" + _arm + "_hand/" + _whichFinger + "/command:o"))
    {
        ret = false;
        cerr << _dbgtag << "Failed to open local fingerController port" << endl;
         _isExplorationValid = false;
    }

    if(!Network::connect("/" + _moduleName + "/" + _arm + "_hand/" + _whichFinger + "/command:o",
                     _fingerControllerPortName))
    {
        cerr << _dbgtag << "Failed to connect" << endl;
        cerr << "/" + _moduleName + "/" + _arm + "_hand/" + _whichFinger + "/command:o" << endl;
        cerr << _fingerControllerPortName << endl << endl;
         _isExplorationValid = false;
    }



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

void ObjectFeaturesThread::setArmController_jnt(yarp::dev::IEncoders *encoder, yarp::dev::IPositionControl *jointCtrl)
{

    _armEncoder = encoder;
    _armJointPositionCtrl = jointCtrl;
}

ObjectFeaturesThread::ObjectFeaturesThread ( int period, ResourceFinder rf ) : RateThread ( period )
{

    // Some sane and safe default values
    _trajectoryTime = 1; // By default take 5 seconds to complete a trajectory
    _maintainContactPeriod = 20;
    _readTactilePeriod = 20;
    _explorationThreadPeriod = 20;
    _isExplorationValid = true; // assume true,

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
    _armEncoder = NULL;
    _armJointPositionCtrl = NULL;

    _proximalJointAngle = 0;
    _proximalJoint_index = 11;

    _zMin = 0;
    _zMax = 0;
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
        Vector pos, orient;
        pos.resize(3);
        orient.resize(4);

        for(int i = 0; i < 3; i++)
            pos[i] = startingPose->get(i).asDouble();
        for(int i = 3; i < 7; i++)
            orient[i-3] = startingPose->get(i).asDouble();

        setStartingPose(pos, orient);

        //_desiredStartingPose_isValid = true;
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
