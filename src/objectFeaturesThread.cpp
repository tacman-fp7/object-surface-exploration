#include <objectFeaturesThread.h>
#include <yarp/os/Network.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>


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
void objectExploration::ObjectFeaturesThread::run()
{
	
	///// Read the tactile data ///////
	Bottle* tactileData = _tactilePort.read(true); // Wait for data
	
	//// Read the corresponding arm position. //////
	///  TODO: this should be changed to the fingertip position ///
	Bottle* armPose = _armPositionPort.read(true);
	if(tactileData->isNull() || armPose->isNull()){
		cerr << "Did not receive tactile or arm data" << endl;
		return;
	}
	
	
	_armPoseMutex.lock();
	for (int i = 0; i < 3; i++)
		_armPosition[i] = armPose->get(i).asDouble();
	for (int i = 3; i < 7; i++)
		_armOrientation[i] = armPose->get(i).asDouble();
	_armPoseMutex.unlock();
	
	//cout << armPose->toString() << endl << endl;
	//cout << _armPosition.toString() << endl;
	//cout << _armOrientation.toString() << endl << endl;
	
	_tactileMutex.lock();
	// The first 12 are for the index finger, I am only using the 4 on the tip
	_tactileSum = tactileData->get(12).asDouble();
	_tactileSum += tactileData->get(13).asDouble();
	_tactileSum += tactileData->get(21).asDouble();
	_tactileSum += tactileData->get(22).asDouble();
	_tactileSum  /= 4;
	_tactileMutex.unlock();
	
	/* cout << "Buffer size:" << tactileData->size() << endl;
	 *  cout << "Tactile data:" << endl;
	 *  for (int i = 12; i < 24; i++)
	 *    cout << tactileData->get(i).toString() << " ";
	 *  cout << endl;
	 *  cout << "Tactile average: " << _tactileSum << endl;
	 */
}



/////////// Accessor and mutators ///////////

Vector objectExploration::ObjectFeaturesThread::getPosition()
{ 
	_armPoseMutex.lock();
	Vector temp = _armPosition;
	_armPoseMutex.unlock();
	return temp;
	
}

bool objectExploration::ObjectFeaturesThread::getHomePose ( Vector& pos, Vector& orient )
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

void objectExploration::ObjectFeaturesThread::setHomePose ( Vector& pos, Vector& orient )
{
	_homePosition = pos;
	_homeOrientation = orient;
	_homePose_isValid = true;
	printPose(pos, orient);
}


void objectExploration::ObjectFeaturesThread::setEndPose ( Vector& pos, Vector& orient )
{
	_desiredEndPosition = pos;
	_desiredEndOrientation = orient;
	_desiredEndPose_isValid = true;
	printPose(pos, orient);
}

void objectExploration::ObjectFeaturesThread::setStartingPose ( Vector& pos, Vector& orient )
{
	_desiredStartingPosition = pos;
	_desiredStartingOrientation = orient;
	_desiredStartingPose_isValid = true;
	printPose(pos, orient);
	
}

bool objectExploration::ObjectFeaturesThread::getDesiredEndPose ( Vector& pos, Vector& orient )
{
	if(_desiredEndPose_isValid)
	{
		pos = _desiredEndPosition;
		orient = _desiredEndOrientation;
	}
	return _desiredEndPose_isValid;
}

void objectExploration::ObjectFeaturesThread::setWayPoint ( Vector pos, Vector orient )
{
	_wayPointPos = pos;
	_wayPointOrient = orient;
	_wayPoint_isValid = true;
	printPose(pos, orient);
}

bool objectExploration::ObjectFeaturesThread::getWayPoint ( Vector& pos, Vector& orient, bool invalidateWayPoint )
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

bool objectExploration::ObjectFeaturesThread::getStartingPose ( Vector& pos, Vector& orient )
{
	if(_desiredStartingPose_isValid)
	{
		pos = _desiredStartingPosition;
		orient = _desiredStartingOrientation;
	}
	return _desiredStartingPose_isValid;
	
}

void objectExploration::ObjectFeaturesThread::printPose ( Vector& pos, Vector& orient )
{
	cout << "Position: " << pos.toString() << endl;
	cout << "Orientation: " << orient.toString() << endl;
}

double objectExploration::ObjectFeaturesThread::getForce()
{
	_tactileMutex.lock();
	double temp = _tactileSum;
	_tactileMutex.unlock();
	
	return temp;
}


Vector objectExploration::ObjectFeaturesThread::getOrientation()
{
	_armPoseMutex.lock();
	Vector temp = _armOrientation;
	_armPoseMutex.unlock();
	return temp;
}



bool objectExploration::ObjectFeaturesThread::threadInit()
{
	yarp::os::RateThread::threadInit();
	
	bool ret = true;
	
	
	
	/////////////////// Connect to the tactile sensor port /////////////////
	if(!_tactilePort.open("/objectExploration/tactileSensors/" + _arm + "_hand")){
		ret = false;
		printf("Failed to open local tactile port\n");
	}
	
	Network::connect("/" + _robotName + "/skin/" + _arm + "_hand_comp",
					 "/objectExploration/tactileSensors/" + _arm + "_hand");
	
	
	/////////////// Opening amr pose port and connecting to it //////////////
	if(!_armPositionPort.open("/objectExploration/" + _arm + "_arm/pose"))
	{
		ret = false;
		cout << "Failed to open local arm pose port" << endl;
	}
	//icubSim/cartesianController/left_arm/state:o
	if(!Network::connect("/" + _robotName + "/" + _controllerName + "/" + _arm + "_arm/state:o",
		"/objectExploration/" + _arm + "_arm/pose"))
	{
		ret = false;
		cerr << "Failed to connect to the arm pose port" << endl;
	}
	
	// TODO: figure out why removing this crashes the application
	// is it because the the network connection needs time?
	if(ret)
		cout << "Object features thread configured" << endl;
	else
		cerr << "Error, object features thread failed during configuration" << endl;
	
	return ret;
}

void objectExploration::ObjectFeaturesThread::threadRelease()
{
	
	_tactilePort.close();
	_armPositionPort.close();
	
}

objectExploration::ObjectFeaturesThread::~ObjectFeaturesThread()
{
	
}

objectExploration::ObjectFeaturesThread::ObjectFeaturesThread ( int period, ResourceFinder rf ) : RateThread ( period )
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
	
	
	_tactileSum = 0;
	_rf = rf;
	
	////////////// read the parameters from the config file ///////////////
	this->readParameters();
}

bool objectExploration::ObjectFeaturesThread::readParameters()
{
	
	
	
	Bottle &robotParameters = _rf.findGroup("RobotParameters");
	if(!robotParameters.isNull()){
		// Read the arm configuration
		_arm = robotParameters.check("arm", Value("left")).asString();
		_robotName = robotParameters.check("robotName", Value("icubSim")).asString();
		_controller = robotParameters.check("controller", Value("Error")).asString();
		_controllerName = robotParameters.check("controllerName", Value("Error")).asString(); 
		_trajectoryTime = robotParameters.check("_trajectoryTime", Value(5)).asInt();
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
	
}

const string& objectExploration::ObjectFeaturesThread::getArm()
{
	return _arm;
}

const string& objectExploration::ObjectFeaturesThread::getControllerName()
{
	return _controllerName;
}

const string& objectExploration::ObjectFeaturesThread::getControllerType()
{
	return _controller;
}

const string& objectExploration::ObjectFeaturesThread::getRobotName()
{
	return _robotName;
}

const int& objectExploration::ObjectFeaturesThread::getTrajectoryTime()
{
	return _trajectoryTime;
}

const int& objectExploration::ObjectFeaturesThread::getExplorationThreadPeriod()
{
	return _explorationThreadPeriod;
}

const int& objectExploration::ObjectFeaturesThread::getMaintainContactPeriod()
{
	return _maintainContactPeriod;
}

const double& objectExploration::ObjectFeaturesThread::getDesiredForce()
{
	return _desiredFroce;
}
