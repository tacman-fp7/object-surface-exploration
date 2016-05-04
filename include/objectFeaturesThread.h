#pragma once
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <string.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/RpcClient.h>
#include <string>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IControlMode2.h>
#include "surfaceModelGP.h"
#include "hand.h"

// This object is used to update features which will be shared between object objectExploraton
// and object classification threads
// I can also add suggested trajectory which can be used by maintain contact module to 
// change trajectory/location

// Make it a thread that reads the finger data, sums 



namespace objectExploration
{

using yarp::os::RateThread;
using yarp::os::BufferedPort;
using yarp::os::Bottle;
using yarp::os::ResourceFinder;
using yarp::sig::Vector;
using yarp::os::Mutex;
using std::string;
using yarp::os::RpcClient;



class ObjectFeaturesThread: public RateThread
{
public:
    ObjectFeaturesThread(int period, ResourceFinder rf);
    ~ObjectFeaturesThread();
    void run();
    bool threadInit();
    void threadRelease();

    void publishContactState(int contactState);
    void publishFingertipPosition(Vector pos);


    void updateContactState(int contactState){
        _contactState = contactState;
        publishContactState(_contactState);}

private:
    void publishFingertipControl(Bottle controlCommand);


protected:
    string _moduleName;
    int _contactState;

    std::string _dbgtag;

    BufferedPort<Bottle> _contactStatePort_out;
    BufferedPort<Bottle> _fingertipPosition_out;
    BufferedPort<Bottle> _fingertipControlPort_out;

};

} // namespace objectExploration
