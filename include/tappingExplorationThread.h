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
    EXCEEDED_ANGLE = 8,
    SAMPLE_SURFACE = 9
};

class TappingExplorationThread: public ExplorationStrategyThread
{
public:
    TappingExplorationThread(int period, ICartesianControl* robotCartesianController,
                            ObjectFeaturesThread* objectFeatures):
        ExplorationStrategyThread(period, robotCartesianController,
                                  objectFeatures){ _nGrid = 0; _forceThreshold = FORCE_TH;
                                                 _curAbduction = -10;
                                                 //_curDistal = -10;
                                                 _curAbduction = -10;}
    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();

protected:
    State _contactState;
    int _repeats;
    double _forceThreshold;
    double _curProximal;
    //double _curDistal;
    double _curAbduction;

private:
    Vector _indexFingerEncoders;
    int _nGrid;
    //Vector _indexFingerPosition;
   // double _preContactForce;

protected:
    //void logFingertipControl();

protected:
    virtual void moveToNewLocation();
    void approachObject();
    virtual void calculateNewWaypoint();
    virtual void maintainContact();
    void finshExploration();
    // Rename the functions to refelect the fact that the proximal and distal are locked
    void moveIndexFinger(double proximalAngle, double abductionAngle, double speed = 40);
    void moveIndexFingerBlocking(double proximalAngle, double abductionAngle, double speed);
    void moveIndexFingerBlocking(double proximalAngle, double distalAngle, double abductionAngle, double speed);

private:
    void moveArmToWayPoint(Vector pos, Vector orient);
    void confrimContact(double maxAngle);
    void detectContact(double maxAngle);


};

} // namespace objectExploration


#endif // TAPPINGEXPLORATIONTHREAD_H
