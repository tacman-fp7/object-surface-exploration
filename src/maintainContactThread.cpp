#include <maintainContactThread.h>
#include <stdio.h>
#include <iostream>
using std::cerr;
using std::cout;
using std::endl;

bool objectExploration::MaintainContactThread::setDesiredForce(double desiredForce)
{
  _desiredForce = desiredForce;
}

void objectExploration::MaintainContactThread::run()
{
  // Read the tactile data
  //cout << _objectFeatures->getForce() << endl;
  // Read the current position
  //cout << _objectFeatures->getPosition().toString() << endl;
  //cout << _objectFeatures->getOrientation().toString() << endl;
  
  // Calculate the action to be taken
  
  // Introduce a new waypoint 
  
}


bool objectExploration::MaintainContactThread::threadInit()
{
  yarp::os::RateThread::threadInit();
  
  if(_objectFeatures == NULL){
   cout << "MaintainContactThread failed: objectFeatures points to NULL, aborting" << endl;
   return false;
  }
  else
  {
      cout << "MaintainContactThread configured" << endl;
  }
  
  return true;
}


void objectExploration::MaintainContactThread::threadRelease()
{
  yarp::os::RateThread::threadRelease();
}
