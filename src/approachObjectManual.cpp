
#include <approachObjectManual.h>
using std::cerr;
using std::endl;
 
objectExploration::ApproachObjectManual::ApproachObjectManual()
{
  
  _contactPose_isValid = false;
  _contactPose_isValid = false;
  _contactPos.resize(POS_SIZE);
  _contactOrient.resize(ORIENT_SIZE);
  _homePos.resize(POS_SIZE);
  _homeOrient.resize(ORIENT_SIZE);
}








bool objectExploration::ApproachObjectManual::approach(yarp::dev::ICartesianControl& armController)
{
  
  if(!_contactPose_isValid)
  {
    cerr << "Error: contact position not initialised" << endl;
    return false;
  }
  // Synched approach
  
  armController.goToPoseSync(_contactPos, _contactOrient);
  //armController.waitMotionDone(0.04);
  
  
  
  return true;
}
