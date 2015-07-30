#include <maintainContactThread.h>
#include <stdio.h>
#include <iostream>
#include <yarp/sig/Vector.h>
using std::cerr;
using std::cout;
using std::endl;
using yarp::sig::Vector;

bool objectExploration::MaintainContactThread::setDesiredForce(double desiredForce)
{
  _desiredForce = desiredForce;
}

void objectExploration::MaintainContactThread::run()
{
  // Read the tactile data
  double force = _objectFeatures->getForce();
  
  // Read the corresponding robot positon
  //TODO: Gotta make sure there is deep copy constructor
  Vector px, po; // position and getOrientation
  px = _objectFeatures->getPosition();
  po = _objectFeatures->getOrientation();
  
  //cout << _objectFeatures->getForce() << endl;
  // Read the current position
  //cout << _objectFeatures->getPosition().toString() << endl;
  //cout << _objectFeatures->getOrientation().toString() << endl;
  
  // Calculate the action to be taken
  
  // Introduce a new waypoint 
  // currently hacking it to an increase in the height
  px[2] += 0.001;
  _objectFeatures->setWayPoint(px, po);
  
  
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
