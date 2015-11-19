#ifndef TAPPINGEXPLORATIONTHREAD_H
#define TAPPINGEXPLORATIONTHREAD_H

#include <explorationStrategyThread.h>


namespace objectExploration
{
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
    // A container for the features
};

} // namespace objectExploration


#endif // TAPPINGEXPLORATIONTHREAD_H
