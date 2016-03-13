#pragma once

#include <string>
#include <yarp/os/ResourceFinder.h>
#include <gurls++/gprwrapper.h>
#include <gurls++/gurls.h>
#include <gurls++/wrapper.h>
#include <gurls++/gmat2d.h>
#include <yarp/sig/Vector.h>
#include <vector>



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
    bool updateModel();
    void saveModelOutput();
    bool updateSurfaceEstimate(const unsigned int nGrid = 120, const double offset = 5/1000);
    //bool saveMeshCSV();

    void loadContactData(const std::string type);
    void addContactPoint(const Vector fingertipPosition);
    void addContactPoint(gVec<double> posXY, gVec<double> posZ);
    void saveContactPoints();
    void padBoundingBox();
    void padBoundingBox(double xMin, double xMax, double yMin, double yMax, double zMin, int nSteps = 5, double offset = 0/1000);
    void setBoundingBox(const unsigned int nPoints = 120, const double offset = 5/1000);
    void setBoundingBox(const double xMin, const double xMax, const double yMin, const double yMax,
                        const unsigned int nPoints = 120, const double offset = 5/1000);

    bool getMaxVariancePose(Vector &maxVariancePos);

private:

    bool init(ResourceFinder& rf);
    double readOption(const string& main, const string& sub,  gurls::GurlsOptionsList *opt);
    gMat2D<double>* eval(const gMat2D<double> &X, gMat2D<double> &vars, gurls::GurlsOptionsList *opt);
    bool getMaxVariancePose(const gMat2D <double> &positions, const gMat2D <double> &variance,
                            const gMat2D <double> &means, Vector &maxVariancePos);
    void printTrainingData();
    //void addPaddingPoints(double startPoint, double endPoint, double constAxis, double targetValue);


private:
    // gurls::GPRWrapper<double>* _gpWrapper;
    std::string _objectName;
    gurls::GURLS _objectModel;
    gurls::GurlsOptionsList* _opt;
    gMat2D<double> _inputTraining;
    gMat2D<double> _outputTraining;
    Vector _maxVariancePos;
    bool _isValidMaxVar;
    string _dbgtg;
    bool _isValidModel;
    gMat2D < double > _inputTesting;
    std::vector <double> _xPoints;
    std::vector <double> _yPoints;
    std::vector <double> _zPoints;
    int _paddingPoints;

    double _maxX, _maxY, _minX, _minY;
}; // end of class
} // end of namespace objectExploration
