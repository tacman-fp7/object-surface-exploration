#ifndef GPEXPLORATION_MULTIFINGER
#define GPEXPLORATION_MULTIFINGER

#include "gpExplorationThread.h"

namespace objectExploration
{
using yarp::os::Mutex;

struct clenchResults{
    bool ringFingerForce;
    bool ringFingerExAngle;
    bool littleFingerForce;
    bool littleFingerExAngle;
};

typedef struct clenchResults clenchResults_t;

class GPExplorationMultifingerThread: public GPExplorationThread
{

public:
    GPExplorationMultifingerThread(int period, Hand* robotHand, Finger* explorationFinger, string objectName,
                        ObjectFeaturesThread* objectFeatures):
        GPExplorationThread(period, robotHand, explorationFinger, objectName,
                                 objectFeatures){
    }

    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();
    void multifingerContact();

protected:
    bool clenchFinger(Finger *finger, double maxAngle);
    bool clenchRingLittleFinger(Finger *ringFinger, Finger *littleFinger, double maxAngle, clenchResults_t *clenchResults);


}; // end of class

} // end of namespace

#endif
