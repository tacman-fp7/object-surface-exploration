#pragma once

#include "finger.h"
#include "simFinger.h"

namespace objectExploration {

class FingerFactory{
public:
    Finger* createFinger(string whichFinger, string whichRobot, t_controllerData ctrlData
                         ){

        ctrlData.whichFinger = whichFinger;


        if(whichRobot.compare("icub") == 0){
            if(whichFinger.compare("index") == 0){
                return new IndexFinger(ctrlData);

            }
            if(whichFinger.compare("middle") == 0){
                return new MiddleFinger(ctrlData);
            }
            if(whichFinger.compare("ring") == 0){
                return new RingFinger(ctrlData);
            }
            if(whichFinger.compare("little") == 0){
                return new LittleFinger(ctrlData);
            }
            else if(whichFinger.compare("thumb") == 0){
                return new Thumb(ctrlData);
            }
        }
        else if(whichRobot.compare("icubSim") == 0){
            if(whichFinger.compare("index") == 0){
                return new SimIndexFinger(ctrlData);
            }
            if(whichFinger.compare("middle") == 0){
                return new SimMiddleFinger(ctrlData);

            }
            else if(whichFinger.compare("thumb") == 0){
                return new SimThumb(ctrlData);

            }
            else if(whichFinger.compare("little") == 0){
                return new SimThumb(ctrlData);
            }
            else if(whichFinger.compare("ring") == 0){
                return new SimThumb(ctrlData);
            }
            else{
                throw std::runtime_error("Finger factory: cannot create icubSim: " + whichFinger );
            }
        }
    }
};


} // end of namespace
