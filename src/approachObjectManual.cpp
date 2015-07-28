
#include <approachObjectManual.h>

 
objectExploration::ApproachObjectManual::ApproachObjectManual()
{
  
  _contactPose_isValid = false;   
  _contactPos.resize(POS_SIZE);
  _contactOrient.resize(ORIENT_SIZE);
  _homePos.resize(POS_SIZE);
  _homeOrient.resize(ORIENT_SIZE);
}








bool objectExploration::ApproachObjectManual::approach(yarp::dev::ICartesianControl& armController)
{
  
  // Synched approach
  armController.goToPoseSync(_contactPos, _contactOrient);
  armController.waitMotionDone(0.04);
  
  
  
  return true;
}
