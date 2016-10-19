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


class ActuatedFingerContactThread: public yarp::os::Thread{
public:

    virtual void run();
    //virtual bool threadInit(){_middleFinger = NULL; _maxAngle = 0; _contactState = false; return true;}
    virtual void threadRelease(){_finger = NULL, _maxAngle = 0;}

    void initThread(Finger *finger, double maxAngle, double forceThreshold);//{_middleFinger = finger; _maxAngle = maxAngle; _forceThreshold = forceThreshold;}
    void getResults(bool *contactState){*contactState = _contactState;}


protected:
    Finger *_finger;
    double _maxAngle;
    bool _contactState;
    double _forceThreshold;


};




class RingAndLittleFingersContactThread: public yarp::os::Thread{

public:
    virtual void run();
    virtual bool threadInit();/*{
                            _clenchResults.littleFingerExAngle = false;
                             _clenchResults.littleFingerForce = false;
                             _clenchResults.ringFingerExAngle = false;
                             _clenchResults.ringFingerForce = false;} */
    virtual void threadRelease(){_ringFinger = NULL; _littleFinger = NULL;}
    void initThread(Finger *ringFinger, Finger *littleFinger, double maxAngle, double forceThreshold){
    _ringFinger = ringFinger; _littleFinger = littleFinger; _maxAngle = maxAngle, _forceThreshold = forceThreshold;
    }
    void getResults(clenchResults_t *clenchResults){
        *clenchResults = _clenchResults;
    }

private:
    Finger *_ringFinger;
    Finger *_littleFinger;
    double _forceThreshold;
    double _maxAngle;
    clenchResults_t _clenchResults;
};

class GPExplorationMultifingerThread: public GPExplorationThread
{

public:
    GPExplorationMultifingerThread(int period, Hand* robotHand, Finger* explorationFinger, Finger* auxiliaryFinger, string objectName,
                        ObjectFeaturesThread* objectFeatures):
        GPExplorationThread(period, robotHand, explorationFinger, auxiliaryFinger, objectName,
                                 objectFeatures){
    }

    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();
    void multifingerContact();

protected:
    bool clenchFinger(Finger *finger, double maxAngle);
    bool clenchRingLittleFinger(Finger *ringFinger, Finger *littleFinger, double maxAngle, clenchResults_t *clenchResults);
    //void logData();
private:
    ActuatedFingerContactThread _contactAuxiliaryFinger;
    RingAndLittleFingersContactThread _contactRingAndLittleFingers;





}; // end of class



} // end of namespace

#endif
