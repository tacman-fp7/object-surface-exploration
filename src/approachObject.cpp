#include <approachObject.h>
#include <string.h> 
#include <iostream>
#include <stdio.h>

using namespace yarp::os;

objectExploration::ApproachObject::ApproachObject()
{
 _contactPose_isValid = false;   
  _contactPos.resize(POS_SIZE);
  _contactOrient.resize(ORIENT_SIZE);
  _homePos.resize(POS_SIZE);
  _homeOrient.resize(ORIENT_SIZE);
}

bool objectExploration::ApproachObject::updateContactpose(Vector& pos, Vector& orient)
{
  _contactPos = pos;
  _contactOrient = orient;
  printf("Contact pose updated %s : %s\n", _contactPos.toString().c_str(), _contactOrient.toString().c_str());
  return true;
}

bool objectExploration::ApproachObject::updateHomePose(Vector& pos, Vector& orient)
{
  _homePos = pos;
  _homeOrient = orient;
  
  printf("Home pose updated %s : %s\n", _homePos.toString().c_str(), _homeOrient.toString().c_str());
  return true;
}


bool objectExploration::ApproachObject::goToHomepose(yarp::dev::ICartesianControl& armController)
{
   // Synched approach
  armController.goToPoseSync(_homePos, _homeOrient);
  armController.waitMotionDone(0.04);
  return true;
}

