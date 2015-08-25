#pragma once
#include <explorationStrategyThread.h>


namespace objectExploration
{
  class PlanarExplorationThread: public ExplorationStrategyThread
  {
  public:
PlanarExplorationThread(int period, ICartesianControl* robotCartesianController,
			ObjectFeaturesThread* objectFeatures):
			ExplorationStrategyThread(period, robotCartesianController, 
                          objectFeatures){}
    void run();
    bool threadInit();
    void threadRelease();
    
  private:
    // A container for the features
  };
  
} // End of namespace
