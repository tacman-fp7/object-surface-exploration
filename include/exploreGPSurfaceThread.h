#pragma once

#include <gpExplorationThread.h>
#include <vector>

#define FORCE_TH 1.6

namespace objectExploration
{

class ExploreGPSurfaceThread: public GPExplorationThread
{

public:
    ExploreGPSurfaceThread(int period, Hand* robotHand, Finger* explorationFinger, string objectName,
                           ObjectFeaturesThread* objectFeatures):
        GPExplorationThread(period, robotHand, explorationFinger, objectName, objectFeatures)
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
    yarp::sig::Vector _nextSamplingPos;
private:
    void moveArmToWayPoint(yarp::sig::Vector pos, yarp::sig::Vector orient);


};
}

