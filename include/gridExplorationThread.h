#pragma once
#include "objectModelGrid.h"
#include "tappingExplorationThread.h"
#include <stdexcept>
#include <contactSafetyThread.h>

namespace objectExploration
{

class GridExplorationThread: public TappingExplorationThread{

public:
    GridExplorationThread(int period, Hand* robotHand, Finger* explorationFinger, Finger* auxiliaryFinger, string objectName,
                            ObjectFeaturesThread* objectFeatures):
            TappingExplorationThread(period, robotHand, explorationFinger, auxiliaryFinger, objectName,
                                     objectFeatures){

        _dbgtag = "GridExplorationThread: ";

        _contactSafetyThread = NULL;
        _objectModel = new ObjectModelGrid(objectName);

        if(_objectModel == NULL){
            throw std::runtime_error(_dbgtag + "could not create object model.");
        }
    }

    void run();
    bool threadInit();
    void threadRelease();

protected:
    virtual void moveToNewLocation();
    virtual void maintainContact();

private:

    ContactSafetyThread* _contactSafetyThread;
    ObjectModelGrid* _objectModel;
    string _dbgtag;
};
} // end of namespace
