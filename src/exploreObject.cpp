#include <exploreObject.h>
#include <approachObjectManual.h>
#include <yarp/sig/Vector.h>
#include <signal.h>
#include <yarp/os/Bottle.h>
#include <planarExplorationThread.h>

using std::cout;
using std::cerr;
using std::endl;
using yarp::os::Value;




objectExploration::ExploreObject::ExploreObject(yarp::os::ResourceFinder& rf)
{
	
	bool failed = false;
	_exploreObjectOnOff = true;
	_stopModule = false;
	_rf = rf;
	
	int readTactilePeriod;
	Bottle& explorationParameters = _rf.findGroup("ExplorationParameters");
	if(!explorationParameters.isNull())
	{
		readTactilePeriod = explorationParameters.check("readTactilePeriod", Value(20)).asInt();
	}
	
	_objectFeaturesThread = new ObjectFeaturesThread(readTactilePeriod,  rf);
	
}

objectExploration::ExploreObject::~ExploreObject()
{
	cout << "Here" << endl;	
	
	if(_maintainContactThread != NULL)
	{
		_maintainContactThread->stop();
		delete(_maintainContactThread);
		_maintainContactThread = NULL;
	}
	cout << "Here2" << endl;	
	if(_objectFeaturesThread != NULL)
	{    
		_objectFeaturesThread->stop();
		delete(_objectFeaturesThread);
		_objectFeaturesThread = NULL;
	}
	cout << "Here3" << endl;	
	
}

bool objectExploration::ExploreObject::goToStartingPose()
{
	Vector pos, orient;
	pos.resize(3);
	orient.resize(4);
	if(_objectFeaturesThread->getStartingPose(pos, orient))
	{
		
		_armCartesianController->goToPoseSync(pos, orient);
		return true;
	}
	
	return false;
	
}



bool objectExploration::ExploreObject::goToHomePose()
{
	Vector pos, orient;
	pos.resize(3); // x,y,z position 
	orient.resize(4); // x,y,z,w prientation

	if(_objectFeaturesThread->getHomePose(pos, orient))
	{
		_armCartesianController->goToPoseSync(pos, orient);
		return true;
	}
	return false;
}

bool objectExploration::ExploreObject::goToEndPose()
{
	Vector pos, orient;
	pos.resize(3); // x,y,z position 
	orient.resize(4); // x,y,z,w prientation
	if(_objectFeaturesThread->getDesiredEndPose(pos, orient))
	{
		_armCartesianController->goToPoseSync(pos, orient); 
		return true;
	}
	return false;
	

}

bool objectExploration::ExploreObject::setStartingPose()
{
	Vector pos, orient;
	pos.resize(3); // x,y,z position 
	orient.resize(4); // x,y,z,w prientation
	_armCartesianController->getPose(pos, orient);
	_objectFeaturesThread->setStartingPose(pos, orient);
	return true;
}

bool objectExploration::ExploreObject::setHomePose()
{
	Vector pos, orient;
	pos.resize(3); // x,y,z position 
	orient.resize(4); // x,y,z,w prientation
	_armCartesianController->getPose(pos, orient);
	_objectFeaturesThread->setHomePose(pos, orient);
	return true;
}

bool objectExploration::ExploreObject::setEndPose()
{
	Vector pos, orient;
	pos.resize(3); // x,y,z position 
	orient.resize(4); // x,y,z,w prientation
	_armCartesianController->getPose(pos, orient);
	_objectFeaturesThread->setEndPose(pos, orient);
	return true;
}

// bool objectExploration::ExploreObject::updateHomePose()
// {
//    Vector pos, orient;
//   pos.resize(3); // x,y,z position 
//   orient.resize(4); // x,y,z,w prientation
//   _armCartesianController->getPose(pos, orient);
//   _approachObjectCntrl->updateHomePose(pos, orient);
// return true;
// }

bool objectExploration::ExploreObject::exploreObject(const bool onOff)
{
	bool ret = true;
	
	
	if(_exploreObjectOnOff)
	{
		//TODO: do some checks if the thread is running on so on
		// First step is to reach the pre-contact location
		if(!this->goToStartingPose())
			ret = false;
		// Then explore the object
		if(!_maintainContactThread->start())
			ret = false;
		
		if(!_exploreObjectThread->start())
			ret = false;
		
		cout << "Exoploring the object\n" << endl;
		_exploreObjectOnOff = false;
		
	}
	else{
		
		if(!this->goToHomePose())
			ret = false;
		
		_maintainContactThread->stop();
		
		_exploreObjectThread->stop();
		
		
		
		cout << "Stopped the exploration" << endl;
		_exploreObjectOnOff = true;
	}
	
	return ret;
}





////////////////// RF module implementation //////////////////


// Attach the port as a server
bool objectExploration::ExploreObject::attach ( yarp::os::Port& source )
{
	
	return this->yarp().attachAsServer(source);
}

bool objectExploration::ExploreObject::configure(yarp::os::ResourceFinder& rf )
{
	
	
	bool ret = true;
	ObjectFeaturesThread& systemParameters = *_objectFeaturesThread; // Just for better naming
	
	/////////////////////////////// Configure the controller //////////////////////////////////
	yarp::os::Property deviceOptions;
	deviceOptions.put("device", systemParameters.getControllerType());
	deviceOptions.put("local", "/client_controller/" + systemParameters.getArm() + "_arm");
	deviceOptions.put("remote", "/" + systemParameters.getRobotName()
	+ "/" + systemParameters.getControllerName() + "/" + systemParameters.getArm() + "_arm");
	
	cout << "Device options: " << deviceOptions.toString() << endl;
	
	if(!_deviceController.open(deviceOptions))
	{
		cerr << "Failed to open the device: " << systemParameters.getControllerType() << endl;
		return false;
	}
	
	/////// Open a Cartesian controller ////////
	
	if(!_deviceController.view(_armCartesianController))
	{
		cerr << "Failed to get a Cartesian view" << endl;
		return false;
	}
	
	//////////////// Configure the Cartesian driver //////////////
	_armCartesianController->setTrajTime(systemParameters.getTrajectoryTime());

	////////// Setting up the tactile data reading thread ////////////
	_objectFeaturesThread->start();
	
	////////// Setting up the MaintainContactThread ///////////////////////
	_maintainContactThread = new MaintainContactThread(systemParameters.getMaintainContactPeriod(),
													   _objectFeaturesThread);
	_maintainContactThread->setDesiredForce(systemParameters.getDesiredForce());
	
	_exploreObjectThread = new PlanarExplorationThread(systemParameters.getExplorationThreadPeriod(), 
													   _armCartesianController,_objectFeaturesThread);
	
	
	// Check if in the config file we have a name for the server
	string moduleName = rf.check("moduleName", Value("robotControlServer"),
								 "module name (string)").asString().c_str();
	
	setName(moduleName.c_str());
	

	

	std::string portName= "/";
	portName+= getName();
	if (!_robotControl_port.open(portName.c_str())) {
	 cout << getName() << ": Unable to open port " << portName << endl;
		return false;
	}
	
	
	this->attach(_robotControl_port);
	
	return ret;
}


bool objectExploration::ExploreObject::close()
{


	// Close neatly, this function is called when Ctl+C is registered
	_robotControl_port.close();
	_deviceController.close();                                // Close the device controller
	
	
	return true;
}

bool objectExploration::ExploreObject::quit()
{
  _stopModule = true;
  return true;
}


bool objectExploration::ExploreObject::updateModule()
{
	
	// Put a repetitive task here that will be run every getPeriod() time
	
	return (!_stopModule);
	
}








