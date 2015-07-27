#pragma once
/*
 * Author: Nawid Jamali
 * Project: TACMAN
 * 
 * Abstract class for iCub to approach an abject
 */
#include <yarp/sig/Vector.h>
#include <yarp/os/ResourceFinder.h>

using  yarp::sig::Vector;
using yarp::os::ResourceFinder;

namespace objectExploration{
  class ApproachObject;
}


class objectExploration::ApproachObject{
  public:
    ApproachObject();
    //bool configure(ResourceFinder& rf);
    virtual bool estimateInitContactPos()=0;
    virtual bool approach()=0;
    
  public:
    Vector _initContactPos;
    
protected:
  bool _initContactPos_isValid;
};

