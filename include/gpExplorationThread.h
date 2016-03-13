#ifndef GPEXPLORATION
#define GPEXPLORATION

#include "tappingExplorationThread.h"
#include "surfaceModelGP.h"

#define FORCE_TH 1.6

namespace objectExploration
{

class GPExplorationThread: public TappingExplorationThread
{

public:
    GPExplorationThread(int period, ICartesianControl* robotCartesianController,
                            ObjectFeaturesThread* objectFeatures):
        TappingExplorationThread(period, robotCartesianController,
                                  objectFeatures){ _surfaceModel = objectFeatures->getGPSurfaceModel();
                                                 _forceThreshold = FORCE_TH;}

    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();
    bool initialiseGP(Vector startingPos, Vector startingOrient,
                      Vector endingPos, Vector endingOrient);

protected:
     void setWayPoint_GP();
     void maintainContact();
     void moveToNewLocation();
     void moveArmUp();


private:
     void makeSingleContact(yarp::sig::Vector pos, yarp::sig::Vector orient);

private:
     SurfaceModelGP *_surfaceModel;


};

} // end of namespace

#endif // GPEXPLORATION

