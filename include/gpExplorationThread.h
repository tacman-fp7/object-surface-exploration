#ifndef GPEXPLORATION
#define GPEXPLORATION

#include "tappingExplorationThread.h"
#include "surfaceModelGP.h"
#include <contactSafetyThread.h>

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
    virtual bool initialiseGP(Vector startingPos, Vector startingOrient,
                      Vector endingPos, Vector endingOrient);

protected:
     virtual void setWayPoint_GP();
     virtual void maintainContact();
     void moveToNewLocation();
     void moveArmUp();
     void sampleSurface_wiggleFingers();

private:
     void makeSingleContact(yarp::sig::Vector pos, yarp::sig::Vector orient);
     void moveArmToWayPoint(yarp::sig::Vector pos, yarp::sig::Vector orient);

protected:
     SurfaceModelGP *_surfaceModel;
     //MaintainContactThread *_maintainContactThread;
     ContactSafetyThread* _contactSafetyThread;
     bool  confrimContact(double maxAngle);


};

} // end of namespace

#endif // GPEXPLORATION

