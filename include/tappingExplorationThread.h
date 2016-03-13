#ifndef TAPPINGEXPLORATIONTHREAD_H
#define TAPPINGEXPLORATIONTHREAD_H

#include <explorationStrategyThread.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

#define FORCE_TH 1.6

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
    SET_WAYPOINT_GP = 7,
    EXCEEDED_ANGLE = 8
};

class TappingExplorationThread: public ExplorationStrategyThread
{
public:
    TappingExplorationThread(int period, ICartesianControl* robotCartesianController,
                            ObjectFeaturesThread* objectFeatures):
        ExplorationStrategyThread(period, robotCartesianController,
                                  objectFeatures){ _nGrid = 0; _forceThreshold = FORCE_TH;}
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

protected:
    void logFingertipControl();

protected:
    virtual void moveToNewLocation();
    void approachObject();
    virtual void calculateNewWaypoint();
    virtual void maintainContact();
    void finshExploration();
    void moveIndexFinger(double angle);
    void moveIndexFingerBlocking(double angle);

private:
    void moveArmToWayPoint(Vector pos, Vector orient);
    void confrimContact(double maxAngle);
    void detectContact(double maxAngle);


};

} // namespace objectExploration


#endif // TAPPINGEXPLORATIONTHREAD_H
