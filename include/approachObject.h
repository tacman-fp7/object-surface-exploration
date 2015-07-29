#pragma once
/*
 * Author: Nawid Jamali
 * Project: TACMAN
 * 
 * Abstract class for iCub to approach an abject
 */
#include <yarp/sig/Vector.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/CartesianControl.h>

using  yarp::sig::Vector;
using yarp::os::ResourceFinder;

#define POS_SIZE 3
#define ORIENT_SIZE 4


namespace objectExploration{
 



class ApproachObject{
  public:
    ApproachObject();
    //bool configure(ResourceFinder& rf);
    virtual bool approach(yarp::dev::ICartesianControl& armController)=0;
    virtual bool goToHomepose(yarp::dev::ICartesianControl& armController);
    virtual bool estimateInitContactPos(){};
    virtual bool updateHomePose(Vector& pos, Vector& orient);
    virtual bool updateContactpose(Vector& pos, Vector& orient);
    
  protected:
    Vector _contactPos;
    Vector _contactOrient;
    Vector _homePos;
    Vector _homeOrient;
    
    
protected:
  bool _contactPose_isValid;
};

}
