#include <exploreObject.h>
#include <approachObjectManual.h>
#include <yarp/sig/Vector.h>
#include <signal.h>

objectExploration::ExploreObject::ExploreObject(yarp::dev::PolyDriver* deviceController)
{

  bool failed = false;
  _deviceController = deviceController;
  
  if(_deviceController->isValid())
  {
    //TODO: Check for successful creation of the object
    if(!_deviceController->view(_armCartesianController))
      printf("Failed to get a cartesian view\n");
    else
      failed = true;
  }
  else{
   printf("The device driver is not valid. Aborting!\n");
   failed = true;
  }
  _approachObjectCntrl = new ApproachObjectManual; 
  
  //if(failed)
  //  raise(SIGINT);
}

objectExploration::ExploreObject::~ExploreObject()
{
  if (_approachObjectCntrl != NULL)
    delete(_approachObjectCntrl);
}

bool objectExploration::ExploreObject::approach()
{
  
  _approachObjectCntrl->approach(*_armCartesianController);
}


bool objectExploration::ExploreObject::goToHomePose()
{
  _approachObjectCntrl->goToHomepose(*_armCartesianController);
}

bool objectExploration::ExploreObject::updateContactPose()
{
  Vector pos, orient;
  pos.resize(3); // x,y,z position 
  orient.resize(4); // x,y,z,w prientation
  _armCartesianController->getPose(pos, orient);
  _approachObjectCntrl->updateContactpose(pos, orient);
  return true;
}

bool objectExploration::ExploreObject::updateHomePose()
{
   Vector pos, orient;
  pos.resize(3); // x,y,z position 
  orient.resize(4); // x,y,z,w prientation
  _armCartesianController->getPose(pos, orient);
  _approachObjectCntrl->updateHomePose(pos, orient);
return true;
}


