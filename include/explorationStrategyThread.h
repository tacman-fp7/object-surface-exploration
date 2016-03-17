#pragma once
#include <yarp/sig/Vector.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>
#include <objectFeaturesThread.h>
#include <yarp/dev/CartesianControl.h>


using yarp::dev::ICartesianControl;

namespace objectExploration
{
class ExplorationStrategyThread: public yarp::os::Thread
{
public:
    ExplorationStrategyThread(int period, ICartesianControl* robotCartesianController,
                              ObjectFeaturesThread* objectFeatures):
        _objectFeatures(objectFeatures), _robotCartesianController(robotCartesianController){}
    
protected:
    ObjectFeaturesThread* _objectFeatures;
    ICartesianControl* _robotCartesianController;

    
    
};

} // namespace objectExploration
