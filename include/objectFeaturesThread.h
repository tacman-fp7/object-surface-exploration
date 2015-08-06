#pragma once
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <string.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Mutex.h>

// This object is used to update features which will be shared between object objectExploraton
// and object classification threads
// I can also add suggested trajectory which can be used by maintain contact module to 
// change trajectory/location

// Make it a thread that reads the finger data, sums 

using yarp::os::RateThread;
using yarp::os::BufferedPort;
using yarp::os::Bottle;
using yarp::os::ResourceFinder;
using yarp::sig::Vector;
using yarp::os::Mutex;
using std::string;


namespace objectExploration
{
	
	
	class ObjectFeaturesThread: public RateThread
	{
	public:
		ObjectFeaturesThread(int period, ResourceFinder rf);
		~ObjectFeaturesThread();
		double getFingerForce(int nFinger){ return _tactileSum;};
		void run();
		bool threadInit();
		void threadRelease();
		//////// accessros and mutators ////
		Vector getPosition();
		Vector getOrientation();
		double getForce();
		void setEndPose(Vector& pos, Vector& orient);
		void setStartingPose(Vector& pos, Vector& orient);
		bool getDesiredEndPose(Vector& pos, Vector& orient);
		bool getStartingPose(Vector& pos, Vector& orient);
		void setHomePose(Vector& pos, Vector& orient);
		bool getHomePose(Vector& pos, Vector& orient);
		void setWayPoint(Vector pos, Vector orient);
		bool getWayPoint(Vector& pos, Vector& orient, bool invalidateWayPoint = true);
		bool readParameters();
		const string& getArm();
		const string& getRobotName();
		const string& getControllerType();
		const string& getControllerName();
		const int& getTrajectoryTime();
		const int& getMaintainContactPeriod();
		const int& getExplorationThreadPeriod();
		const double& getDesiredForce();
		
	private:
		void printPose(Vector& pos, Vector& prient);
	protected:
		ResourceFinder _rf;
		
		/////// Robot parameters ///////////
		string _arm;
		string _robotName;
		string _controller;
		string _controllerName;
		int _trajectoryTime;
		
		////// Exploration parameters ///////
		int _maintainContactPeriod;
		int _readTactilePeriod;
		int _explorationThreadPeriod;
		
		double _desiredFroce;
		Mutex _desiredForceMutex;
		
		
		bool _desiredStartingPose_isValid;
		Vector _desiredStartingPosition;
		Vector _desiredStartingOrientation;
		
		bool _desiredEndPose_isValid;
		Vector _desiredEndPosition;
		Vector _desiredEndOrientation;
		
		bool _homePose_isValid;
		Vector _homePosition;
		Vector _homeOrientation;
		
		//////// Object Features ////////////
		BufferedPort<Bottle> _tactilePort;
		BufferedPort<Bottle> _armPositionPort; 
		Mutex _tactileMutex;
		double _tactileSum; 
		
		
		/// Clean them a little later /////
		
		Mutex _armPoseMutex;
		Vector _armPosition;
		Vector _armOrientation;
		
		bool _wayPoint_isValid;
		Vector _wayPointPos;
		Vector _wayPointOrient;
		
		
	};
	
}                                                           // End of namespace