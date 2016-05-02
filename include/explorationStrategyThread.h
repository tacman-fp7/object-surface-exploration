#pragma once
#include <yarp/sig/Vector.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>
#include <objectFeaturesThread.h>
#include <yarp/dev/CartesianControl.h>
#include "hand.h"
#include "finger.h"

using yarp::dev::ICartesianControl;

namespace objectExploration
{




class ExplorationStrategyThread: public yarp::os::Thread
{
public:
    ExplorationStrategyThread(int period, Hand* robotHand, Finger* explorationFinger,
                              ObjectFeaturesThread* objectFeatures):
        _objectFeatures(objectFeatures), _robotHand(robotHand), _explorationFinger(explorationFinger){}
    
protected:
    ObjectFeaturesThread* _objectFeatures;
    //ICartesianControl* _robotCartesianController;
    Hand* _robotHand;
    Finger* _explorationFinger;

    
    
};

} // namespace objectExploration
