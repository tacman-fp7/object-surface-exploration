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
    STOP = 6
};

class TappingExplorationThread: public ExplorationStrategyThread
{
public:
    TappingExplorationThread(int period, ICartesianControl* robotCartesianController,
                            ObjectFeaturesThread* objectFeatures):
        ExplorationStrategyThread(period, robotCartesianController,
                                  objectFeatures){_preContactForce = 0; _nGrid = 0;}
    void run();
    bool threadInit();
    void threadRelease();

private:
    State _contactState;
    Vector _indexFingerEncoders;
    int _nGrid;
    //Vector _indexFingerPosition;
    double _preContactForce;

private:
    void moveToNewLocation();
    void approachObject();
    void calculateNewWaypoint();
    void maintainContact();
    void finshExploration();


};

} // namespace objectExploration


#endif // TAPPINGEXPLORATIONTHREAD_H
