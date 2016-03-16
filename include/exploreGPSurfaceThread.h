#pragma once

#include <gpExplorationThread.h>
#include <vector>

#define FORCE_TH 1.6

namespace objectExploration
{

class ExploreGPSurfaceThread: public GPExplorationThread
{

public:
    ExploreGPSurfaceThread(int period, ICartesianControl* robotCartesianController,
                           ObjectFeaturesThread* objectFeatures):
        GPExplorationThread(period, robotCartesianController, objectFeatures)
    {
        _surfaceModel = objectFeatures->getGPSurfaceModel();
        _forceThreshold = FORCE_TH;
        _wayPointListComplete = false;
    }

    virtual void run();
    //virtual bool threadInit();
    //virtual void threadRelease();

    virtual bool initialiseGP(Vector startingPos, Vector startingOrient,
                      Vector endingPos, Vector endingOrient);


protected:
    virtual void setWayPoint_GP();
    virtual void maintainContact();

private:
    std::vector < yarp::sig::Vector > _wayPointList;
    bool _wayPointListComplete;

private:
    void moveArmToWayPoint(yarp::sig::Vector pos, yarp::sig::Vector orient);

};
}

