#include <approachObject.h>
#include <string.h> 
#include <iostream>
#include <stdio.h>

using namespace yarp::os;
using std::cout;
using std::cerr;
using std::endl;

objectExploration::ApproachObject::ApproachObject()
{
 _contactPose_isValid = false; 
 _homePose_isValid = false;
 _endPose_isValid = false;
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
  _contactPose_isValid = true;
  return true;
}

bool objectExploration::ApproachObject::updateHomePose(Vector& pos, Vector& orient)
{
  _homePos = pos;
  _homeOrient = orient;
  
  printf("Home pose updated %s : %s\n", _homePos.toString().c_str(), _homeOrient.toString().c_str());
  _homePose_isValid = true;
  return true;
}

bool objectExploration::ApproachObject::setEndPose(Vector& pos, Vector& orient)
{
  _endPos = pos;
  _endOrient = orient;
  printf("End pose set %s : %s\n", _endPos.toString().c_str(), _endOrient.toString().c_str());
  _endPose_isValid = true;
}


bool objectExploration::ApproachObject::goToHomepose(yarp::dev::ICartesianControl& armController)
{
   // Synched approach
  if(!_homePose_isValid){
      cerr << "Error, home-pose is not initialised" << endl;
      return false;
  }
  // We are safe to move the arm
  armController.goToPoseSync(_homePos, _homeOrient);
  armController.waitMotionDone(0.04);
  return true;
}

bool objectExploration::ApproachObject::goToEndPose(yarp::dev::ICartesianControl& armController)
{

  if(!_endPose_isValid)
  {
    cerr << "Error, end-pose is not initialised" << endl;
    return false;
  }
   armController.goToPoseSync(_endPos, _endOrient);
   return true;
}

