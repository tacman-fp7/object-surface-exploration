#pragma once
#include "finger.h"

namespace objectExploration {

class simFinger:public Finger{
protected:
    simFinger(t_controllerData ctrlData);
};


class SimIndexFinger: public Finger{
public:
    SimIndexFinger(t_controllerData ctrlData);
    bool calibrate(){ return true;}
    bool prepare();
    virtual bool setSynchroProximalAngle(double proximal);
    //void getRawTactileData(Vector rawTactileData);
};

class SimMiddleFinger: public Finger{
public:
    SimMiddleFinger(t_controllerData ctrlData);
    bool calibrate(){return true;}
    bool prepare();
    bool  setSynchroProximalAngle(double proximal);
    //void getTawTactileData(Vector rawTactileData);
};

class SimThumb: public Finger{
public:
    SimThumb(t_controllerData ctrlData);
    bool prepare();
};


}// end of namespace
