#ifndef TAPPINGEXPLORATIONTHREAD_H
#define TAPPINGEXPLORATIONTHREAD_H

#include <explorationStrategyThread.h>
#include <yarp/sig/Vector.h>

namespace objectExploration
{


enum State{
    UNDEFINED,
    APPROACH_OBJECT,
    CALCULATE_NEWWAYPONT,
    MAINTAIN_CONTACT,
    MOVE_LOCATION,
    FINISHED,
    STOP
};

class TappingExplorationThread: public ExplorationStrategyThread
{
public:
    TappingExplorationThread(int period, ICartesianControl* robotCartesianController,
                            ObjectFeaturesThread* objectFeatures):
        ExplorationStrategyThread(period, robotCartesianController,
                                  objectFeatures){}
    void run();
    bool threadInit();
    void threadRelease();

private:
    State _contactState;
    Vector _indexFingerEncoders;
private:
    void moveToNewLocation();
    void approachObject();
    void calculateNewWaypoint();
    void maintainContact();
    void finshExploration();

};

} // namespace objectExploration


#endif // TAPPINGEXPLORATIONTHREAD_H
