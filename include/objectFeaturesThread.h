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

/*enum fingerJoints{
    ABDUCTION = 7,
    THUMBT_PROXIMAL = 9,
    THUMB_DISTAL = 10,
    INDEX_PROXIMAL = 11,
    INDEX_DISTAL = 12
};*/

/*struct reachableSpace
{
    double minX;
    double maxX;
    double minY;
    double maxY;
    double minZ;
    double maxZ;
    double disasterX;
};*/

//typedef struct reachableSpace reachableSpace;

class ObjectFeaturesThread: public RateThread
{
public:
    ObjectFeaturesThread(int period, ResourceFinder rf);
    ~ObjectFeaturesThread();
    void run();
    bool threadInit();
    void threadRelease();


    double getContactForce();



    void setEndPose(Vector& pos, Vector& orient);
    void setStartingPose(Vector& pos, Vector& orient);
    bool getDesiredEndPose(Vector& pos, Vector& orient);
    bool getStartingPose(Vector& pos, Vector& orient);
    bool getEndingPose(Vector& pos, Vector& orient);

    bool setWayPoint(Vector pos, Vector orient);
    bool setWayPointGP(Vector pos, Vector orient);
    bool getWayPoint(Vector& pos, Vector& orient, bool invalidateWayPoint = true);


    const string& getArm();
    const string& getRobotName();


    const int& getExplorationThreadPeriod();

    void setArmController_jnt(yarp::dev::IEncoders *encoder, yarp::dev::IPositionControl *jointCtrl);
    void setArmController_cart(yarp::dev::ICartesianControl * cartesianCtrl);
    void setArmController_mode(yarp::dev::IControlMode2 *armJointCtrlmode);
    bool isExplorationValid(){return _isExplorationValid;}
    //double getProximalJointAngle(){return _proximalJointAngle;}


    void publishContactState(int contactState);
    void publishFingertipPosition(Vector pos);

    bool prepGP();

    //bool moveArmToPosition(Vector pos, Vector orient);
    bool fingerMovePosition(int joint, double angle, double speed = 40); /////
    void updateContactState(int contactState){_contactState = contactState;

                                              publishContactState(_contactState);}



    objectExploration::SurfaceModelGP* getGPSurfaceModel(){
        return _objectSurfaceModelGP;
    }

    //bool checkOpenHandDone();



private:
    void printPose(Vector& pos, Vector& prient);
    void updateRobotReachableSpace();
    void publishFingertipControl(Bottle controlCommand);
    bool readParameters();

protected:
    ResourceFinder _rf;

    /////// Robot parameters ///////////
    string _arm;
    string _robotName;
    string _controller;
    string _controllerName;
    string _moduleName;
    string _whichFinger;

//    int _trajectoryTime;




    int _contactState;

    ////// Exploration parameters ///////
    //int _maintainContactPeriod;
    //int _readTactilePeriod;
    int _explorationThreadPeriod;
    bool _isExplorationValid;

   //double _desiredFroce;
   // Mutex _desiredForceMutex;


    bool _desiredStartingPose_isValid;
    Vector _desiredStartingPosition;
    Vector _desiredStartingOrientation;
    //double _zMax;
    //double _zMin;

    bool _desiredEndPose_isValid;
    Vector _desiredEndPosition;
    Vector _desiredEndOrientation;

    bool _homePose_isValid;
    Vector _homePosition;
    Vector _homeOrientation;

    //////// Object Features ////////////

    Mutex _tactileMutex;
    BufferedPort<Bottle> _contactForceCoPPort;
    double _contactForce;




    /// Clean them a little later /////

    ///
    Mutex _armJointMutex;
    Vector _armJoints;

    /// Arm pose and orient is actually gripper pose and orient!!!!
    Mutex _armPoseMutex;
    Vector _armPosition;
    Vector _armOrientation;

    bool _wayPoint_isValid;
    Vector _wayPointPos;
    Vector _wayPointOrient;

    std::string _dbgtag;

    //// The port to read the joint information


    yarp::dev::IPositionControl *_armJointPositionCtrl;



    BufferedPort<Bottle> _contactStatePort_out;
    BufferedPort<Bottle> _fingertipPosition_out;
    BufferedPort<Bottle> _fingertipControlPort_out;



    ////
    objectExploration::SurfaceModelGP *_objectSurfaceModelGP;


    reachableSpace _robotReachableSpace;


};

} // namespace objectExploration
