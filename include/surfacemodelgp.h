#pragma once

#include <string>
#include <yarp/os/ResourceFinder.h>
#include <gurls++/gprwrapper.h>
#include <gurls++/gurls.h>
#include <gurls++/wrapper.h>
#include <gurls++/gmat2d.h>




namespace objectExploration
{

using yarp::os::ResourceFinder;
using std::string;
using gurls::gMat2D;


class SurfaceModelGP
{
public:
    SurfaceModelGP();
    ~SurfaceModelGP();
    bool trainModel(); // with some sort of input
    bool loadModel(); // from the disk
    bool saveModel(); //to the disk
    bool saveMeshCSV();
    void loadContactData();

private:
    bool init(ResourceFinder& rf);
    double readOption(const string& main, const string& sub,  gurls::GurlsOptionsList *opt);
    gMat2D<double>* eval(const gMat2D<double> &X, gMat2D<double> &vars, gurls::GurlsOptionsList *opt);

private:
   // gurls::GPRWrapper<double>* _gpWrapper;
    gurls::GURLS _objectModel;
    gurls::GurlsOptionsList* opt;
    gMat2D<double> _inputTraining;
    gMat2D<double> _outputTraining;
    string _dbgtg;

}; // end of class
} // end of namespace objectExploration
