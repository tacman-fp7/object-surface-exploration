#pragma once
#include <yarp/os/Thread.h>
#include <objectFeaturesThread.h>
#include "hand.h"
#include "finger.h"
#include <contactSafetyThread.h>

//#define FORCE_TH 1
namespace objectExploration{

class ExplorationStrategyThread: public yarp::os::Thread
{
public:
    ExplorationStrategyThread(int period, Hand* robotHand, Finger* explorationFinger, Finger* auxiliaryFinger,
                              string objectName, ObjectFeaturesThread* objectFeatures):
        _objectFeatures(objectFeatures), _robotHand(robotHand), _explorationFinger(explorationFinger),
    _auxiliaryFinger(auxiliaryFinger){}

    void setContactSafetyForceThreshold(const double threshold){
        if(_contactSafetyThread != NULL){
            _contactSafetyThread->setForceThreshold(threshold);
        }
    }

    volatile void setNRepeats(const int nRepeats){}
//    void setContactForceThreshold(const double threshold){_contactForceThreshold = threshold;}
    
protected:
    ObjectFeaturesThread* _objectFeatures;
    Hand* _robotHand;
    Finger* _explorationFinger;
    Finger* _auxiliaryFinger;

    SurfaceModel* _surfaceModel;

    std::ofstream _explorationFingerLog;
    std::ofstream _auxiliaryFingerLog;
    std::ofstream _explorationFingerTaxelResponseLog;

protected:
    ContactSafetyThread* _contactSafetyThread;
    //double _contactForceThreshold; // Force threshold to determine contact

};

} // namespace objectExploration
