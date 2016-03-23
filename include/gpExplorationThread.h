#ifndef GPEXPLORATION
#define GPEXPLORATION

#include "tappingExplorationThread.h"
#include "surfaceModelGP.h"
#include <contactSafetyThread.h>
#include <yarp/os/Mutex.h>

#define FORCE_TH 1.6

namespace objectExploration
{
using yarp::os::Mutex;

class GPExplorationThread: public TappingExplorationThread
{

public:
    GPExplorationThread(int period, ICartesianControl* robotCartesianController,
                            ObjectFeaturesThread* objectFeatures):
        TappingExplorationThread(period, robotCartesianController,
                                  objectFeatures){ _surfaceModel = objectFeatures->getGPSurfaceModel();
                                                 _forceThreshold = FORCE_TH; _sampleSurface = true;
                                                 _refineModel = false; _validatePositionsEnabled = false;}

    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();
    virtual bool initialiseGP(Vector startingPos, Vector startingOrient,
                      Vector endingPos, Vector endingOrient);
    void enableSurfaceSampling();
    void disableSurfaceSampling();
    void enableRefiningModel(){ _stateMutex.lock(); _refineModel = true; _stateMutex.unlock();}
    void disableRefiningModel(){ _stateMutex.lock(); _refineModel = false;_stateMutex.unlock();}
    void enableValidatePositions(){ _stateMutex.lock(); _validatePositionsEnabled = true;_stateMutex.unlock();}
    void disaleValidatePositions(){ _stateMutex.lock(); _validatePositionsEnabled = false;_stateMutex.unlock();}

protected:
     virtual void setWayPoint_GP();
     virtual void maintainContact();
     void moveToNewLocation();
     void moveArmUp();
     void sampleSurface_wiggleFingers();
     void setWayPoint_GP_Refine();
     void setWayPoint_GP_validate();
     void maintainContact_GP_Refine();
     void maintainContact_GP_validatePosition();


private:
     void makeSingleContact(yarp::sig::Vector pos, yarp::sig::Vector orient);
     void moveArmToWayPoint(yarp::sig::Vector pos, yarp::sig::Vector orient);

protected:
     SurfaceModelGP *_surfaceModel;
     //MaintainContactThread *_maintainContactThread;
     ContactSafetyThread* _contactSafetyThread;
     bool  confrimContact(double maxAngle);
     bool _sampleSurface;

protected:
     Mutex _stateMutex;
     bool _refineModel;
     bool _validatePositionsEnabled;


};

} // end of namespace

#endif // GPEXPLORATION

