#ifndef TAPPINGEXPLORATIONTHREAD_H
#define TAPPINGEXPLORATIONTHREAD_H

#include <explorationStrategyThread.h>


namespace objectExploration
{


enum State{
    UNDEFINED,
    APPROACHING,
    INCONTACT,
    MOVELOCATION
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
private:
    void moveToNewLocation();
};

} // namespace objectExploration


#endif // TAPPINGEXPLORATIONTHREAD_H
