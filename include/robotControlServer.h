#include <yarp/os/RFModule.h>
#include <robotControl.h>

class robotControlServer: public robotControl, public yarp::os::RFModule
{
public:
  robotControlServer();
  virtual bool approach();
  virtual bool contact();
  virtual bool explore();
  virtual bool quit();
  
  // RF module methods
  virtual bool attach(yarp::os::Port &source);
  virtual bool configure( yarp::os::ResourceFinder &rf );
  virtual bool updateModule();
  virtual bool close();
  
private:
  yarp::os::Port _port;
  bool _stopModule;
  
};

