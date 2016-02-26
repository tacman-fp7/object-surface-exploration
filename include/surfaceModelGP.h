#pragma once

#include <string>
#include <yarp/os/ResourceFinder.h>
#include <gurls++/gprwrapper.h>
#include <gurls++/gurls.h>
#include <gurls++/wrapper.h>
#include <gurls++/gmat2d.h>
#include <yarp/sig/Vector.h>



namespace objectExploration
{

using yarp::os::ResourceFinder;
using std::string;
using gurls::gMat2D;
using gurls::gVec;
using yarp::sig::Vector;

class SurfaceModelGP
{
public:
    SurfaceModelGP(const std::string objectName);
    ~SurfaceModelGP();
    bool trainModel(); // with some sort of input
    bool loadModel(); // from the disk
    bool saveModel(); //to the disk
    bool saveMeshCSV();
    void loadContactData();
    void loadContactData(const std::string fileName);
    void addContactPoint(const Vector &fingertipPosition);
    void addContactPoint(gVec<double> posXY, gVec<double> posZ);
    void saveContactPoints(const std::string &fileName);
    //bool getMaxVariancePose(yarp::sig::Vector pos);

    bool getMaxVariancePose(Vector &maxVariancePos);
    bool getMaxVariancePose(const gMat2D <double> &positions,
                            const gMat2D <double> &variance,
                            const gMat2D <double> &means,
                            Vector &maxVariancePos);
private:
    bool init(ResourceFinder& rf);
    double readOption(const string& main, const string& sub,  gurls::GurlsOptionsList *opt);
    gMat2D<double>* eval(const gMat2D<double> &X, gMat2D<double> &vars, gurls::GurlsOptionsList *opt);


private:
    // gurls::GPRWrapper<double>* _gpWrapper;
    std::string _objectName;
    gurls::GURLS _objectModel;
    gurls::GurlsOptionsList* _opt;
    gMat2D<double> _inputTraining;
    gMat2D<double> _outputTraining;
    string _dbgtg;
    bool _isValidModel;
}; // end of class
} // end of namespace objectExploration
