#ifndef TAPPINGEXPLORATIONTHREAD_H
#define TAPPINGEXPLORATIONTHREAD_H

#include <explorationStrategyThread.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

namespace objectExploration
{


enum State{
    UNDEFINED = 0,
    APPROACH_OBJECT = 1,
    CALCULATE_NEWWAYPONT = 2,
    MAINTAIN_CONTACT = 3,
    MOVE_LOCATION = 4,
    FINISHED = 5,
    STOP = 6,
    SET_WAYPOINT_GP = 7
};

class TappingExplorationThread: public ExplorationStrategyThread
{
public:
    TappingExplorationThread(int period, ICartesianControl* robotCartesianController,
                            ObjectFeaturesThread* objectFeatures):
        ExplorationStrategyThread(period, robotCartesianController,
                                  objectFeatures){ _nGrid = 0;}
    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();

protected:
    State _contactState;
    int _repeats;
    double _forceThreshold;
    double _curProximal;
    double _curDistal;

private:
    Vector _indexFingerEncoders;
    int _nGrid;
    //Vector _indexFingerPosition;
   // double _preContactForce;

private:
    void logFingertipControl();

protected:
    virtual void moveToNewLocation();
    void approachObject();
    virtual void calculateNewWaypoint();
    virtual void maintainContact();
    void finshExploration();


};

} // namespace objectExploration


#endif // TAPPINGEXPLORATIONTHREAD_H
