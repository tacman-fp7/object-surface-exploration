#include <maintainContactThread.h>
#include <stdio.h>

bool objectExploration::MaintainContactThread::setDesiredForce(double desiredForce)
{
  _desiredForce = desiredForce;
}

void objectExploration::MaintainContactThread::run()
{
  // Read the tactile data
  
  
  // request a move to the next location
}


bool objectExploration::MaintainContactThread::threadInit()
{
  yarp::os::RateThread::threadInit();
  
  
}


void objectExploration::MaintainContactThread::threadRelease()
{
  yarp::os::RateThread::threadRelease();
}
