#ifndef TAPPINGEXPLORATIONTHREAD_H
#define TAPPINGEXPLORATIONTHREAD_H

#include <explorationStrategyThread.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

#define FORCE_TH 0.8

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
    SAMPLE_SURFACE = 9,
    REFINE_CONTACT = 10,
    VALIDATE_CONTACT = 11
};

class TappingExplorationThread: public ExplorationStrategyThread
{
public:
    TappingExplorationThread(int period, Hand *robotHand, Finger *explorationFinger, string objectName,
                            ObjectFeaturesThread* objectFeatures);


    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();
    /*
     * Set the number of times a contact location is sampled
     */
    void setNRepeats(int nRepeats);

protected:
    State _contactState;
    int _repeats; // Number of contact repeats so far
    int _nRepeats; // Desired number of repeats
    double _forceThreshold; // Force threshold to determine contact
    double _curProximal;  // Used for logging
    double _curAbduction; // Used for logging


    int _nGrid; // should be moved to grid exploration

private:
    Vector _indexFingerEncoders;



protected:
    void moveArmUp();
    virtual void moveToNewLocation();
    void calculateNewWaypoint();
    virtual void maintainContact();

    void approachObject(); 
    bool confrimContact(double maxAngle);
    void finshExploration();

    // Rename the functions to refelect the fact that the proximal and distal are locked
    void moveExplorationFinger(double proximalAngle, double abductionAngle, double speed = 40);
    void moveExplorationFingerBlocking(double proximalAngle, double abductionAngle, double speed);
    void moveExplorationFingerBlocking(double proximalAngle, double distalAngle, double abductionAngle, double speed);

    void moveFinger(Finger *finger, double proximalAngle, double abductionAngle, double speed = 40);
    void moveFingerBlocking(Finger *finger, double proximalAngle, double abductionAngle, double speed);
    void moveFingerBlocking(Finger *finger, double proximalAngle, double distalAngle, double abductionAngle, double speed);

private:
    void moveArmToWayPoint(Vector pos, Vector orient);
    void detectContact(double maxAngle);
    yarp::os::RpcClient _skinManagerCommand;
     BufferedPort<Bottle> _contactStatePort_out;


};

} // namespace objectExploration


#endif // TAPPINGEXPLORATIONTHREAD_H
