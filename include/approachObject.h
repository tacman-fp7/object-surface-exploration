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
#include <objectFeaturesThread.h>

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
    virtual bool setEndPose(Vector& pos, Vector& orient);
    virtual bool goToEndPose(yarp::dev::ICartesianControl& armController);
    
private:
  ObjectFeaturesThread* objectFeatures;
  protected: 
    // Starting pose
    Vector _contactPos; 
    Vector _contactOrient;
    // Home pose
    Vector _homePos;
    Vector _homeOrient;
    
    // End pose
    Vector _endPos;
    Vector _endOrient;
    
protected:
  bool _contactPose_isValid;
  bool _homePose_isValid;
  bool _endPose_isValid;
};

}
