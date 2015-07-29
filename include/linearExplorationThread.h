#pragma once
#include <explorationStrategyThread.h>

namespace objectExploration
{
  class LinearExplorationThread: public ExplorationStrategyThread
  {
  public:
    yarp::sig::Vector getNextWayPoint(){/*Do nothing at the moment*/};
    
  private:
    // A container for the features
  };
  
} // End of namespace