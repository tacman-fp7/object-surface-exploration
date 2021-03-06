#ifndef GPEXPLORATION
#define GPEXPLORATION

#include "tappingExplorationThread.h"
#include "surfaceModelGPActive.h"
#include <contactSafetyThread.h>
#include <yarp/os/Mutex.h>
#include <vector>

//#define FORCE_TH 1.6

namespace objectExploration
{
using yarp::os::Mutex;

class GPExplorationThread: public TappingExplorationThread
{

public:
    GPExplorationThread(int period, Hand* robotHand, Finger* explorationFinger, Finger* auxiliaryFinger, string objectName,
                        ObjectFeaturesThread* objectFeatures):
        TappingExplorationThread(period, robotHand, explorationFinger, auxiliaryFinger, objectName,
                                 objectFeatures){
        _surfaceModel = new SurfaceModelGPActive(objectName);
        _forceThreshold = FORCE_TH;
        _sampleSurface = true;
        _refineModel = false;
        _validatePositionsEnabled = false;
        _contactSafetyThread = NULL;
    }

    ~GPExplorationThread(){

        if(_surfaceModel != NULL){
            delete _surfaceModel;
            _surfaceModel = NULL;
        }
    }

    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();
    virtual bool initialiseGP(Vector startingPos, Vector startingOrient,
                              Vector endingPos, Vector endingOrient);
    void enableSurfaceSampling();
    void disableSurfaceSampling();
    void enableRefiningModel(){ _stateMutex.lock(); _refineModel = true; _stateMutex.unlock();}
    void disableRefiningModel(){ _stateMutex.lock(); _refineModel = false;_stateMutex.unlock();}
    void enableValidatePositions(){ _stateMutex.lock(); _validatePositionsEnabled = true; std::cout << "validate enabled" << std::endl;
                                    _stateMutex.unlock();}
    void disaleValidatePositions(){ _stateMutex.lock(); _validatePositionsEnabled = false;_stateMutex.unlock();}

protected:
    virtual void setWayPoint_GP();
    virtual void maintainContact();
    void moveToNewLocation();
    //void moveArmUp();
    void sampleSurface_wiggleFingers();
    void setWayPoint_GP_Refine();
    void setWayPoint_GP_validate();
    void maintainContact_GP_Refine();
    void maintainContact_GP_validatePosition();


private:
    void makeSingleContact(yarp::sig::Vector pos, yarp::sig::Vector orient);
    void moveArmToWayPoint(yarp::sig::Vector pos, yarp::sig::Vector orient);
    double getMedian(std::vector<double> &vec);

protected:
    //SurfaceModelGP *_surfaceModel;
    //MaintainContactThread *_maintainContactThread;
    //ContactSafetyThread* _contactSafetyThread;
    bool  confirmWiggleContact(double maxAngle);
    bool _sampleSurface;

protected:
    Mutex _stateMutex;
    bool _refineModel;
    bool _validatePositionsEnabled;

private:
    std::vector<double> _minZPoints;
    std::vector<double> _zPoints;
    //yarp::os::RpcClient _skinManagerCommand;
    yarp::os::BufferedPort<Bottle> _tactileData_in;

};

} // end of namespace

#endif // GPEXPLORATION

