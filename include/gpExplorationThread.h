#ifndef GPEXPLORATION
#define GPEXPLORATION

#include "tappingExplorationThread.h"
#include "surfaceModelGP.h"

namespace objectExploration
{

class GPExplorationThread: public TappingExplorationThread
{

public:
    GPExplorationThread(int period, ICartesianControl* robotCartesianController,
                            ObjectFeaturesThread* objectFeatures):
        TappingExplorationThread(period, robotCartesianController,
                                  objectFeatures){}

    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();

protected:
     void setWayPoint_GP();
     void maintainContact();
     void moveToNewLocation();

private:
     SurfaceModelGP *_surfaceModel;


};

} // end of namespace

#endif // GPEXPLORATION

