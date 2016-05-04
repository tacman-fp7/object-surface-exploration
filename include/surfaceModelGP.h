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

class SurfaceModel{
public:

    //SurfaceModel(){}
    //SurfaceModel(const std::string objectName);
    //~SurfaceModel();
    virtual bool trainModel() = 0; // with some sort of input
   //virtual bool loadModel(); // from the disk
    //virtual bool saveModel(); //to the disk
    virtual bool updateModel() = 0;
    //virtual void saveModelOutput();
    virtual bool updateSurfaceEstimate(const unsigned int nGrid = 120, const double offset = 0.0/1000) = 0;


    virtual void loadContactData(const std::string type) = 0;
    virtual void addContactPoint(const Vector fingertipPosition) = 0;
    virtual void addContactPoint(gVec<double> posXY, gVec<double> posZ) = 0;
    virtual void saveContactPoints() = 0;
    virtual void padBoundingBox() = 0;
    virtual void padBoundingBox(double xMin, double xMax, double yMin, double yMax, double zMin, int nSteps = 5, double offset = 0.0/1000) = 0;
    virtual void setBoundingBox(const unsigned int nPoints = 120, const double offset = 0.0/1000) = 0;
    virtual void setBoundingBox(const double xMin, const double xMax, const double yMin, const double yMax,
                        const unsigned int nPoints = 120, const double offset = 0.0/1000) = 0;

    virtual bool getMaxVariancePose(Vector &maxVariancePos) = 0;
    virtual bool getMaxEstimatePos(Vector &maxEstimatePos) = 0;
    virtual bool getNextSamplingPosition(Vector &nextSamplingPosition, bool nextRow = false) = 0;
    virtual bool getNextRefinementPosition(Vector &nextSamplingPosition) = 0;
    virtual bool getNextValidationPosition(Vector &validationPosition) = 0;
    virtual void enableRefinement() = 0;
    virtual void disableRefinement() = 0;
    virtual void enableValidation() = 0;
    virtual void disableValidation() = 0;
    virtual bool validatePosition(Vector &validationPosition) = 0;


};

class SurfaceModelGP: public SurfaceModel{
public:
    SurfaceModelGP(const std::string objectName);
    ~SurfaceModelGP();
    bool trainModel(); // with some sort of input
    //bool loadModel(); // from the disk
    //bool saveModel(); //to the disk
    bool updateModel();
    //void saveModelOutput();
    //void clearModel();
    bool updateSurfaceEstimate(const unsigned int nGrid = 120, const double offset = 0.0/1000);
    //bool saveMeshCSV();

    void loadContactData(const std::string type);
    void addContactPoint(const Vector fingertipPosition);
    void addContactPoint(gVec<double> posXY, gVec<double> posZ);
    void saveContactPoints();
    void padBoundingBox();
    void padBoundingBox(double xMin, double xMax, double yMin, double yMax, double zMin, int nSteps = 5, double offset = 0.0/1000);
    void setBoundingBox(const unsigned int nPoints = 120, const double offset = 0.0/1000);
    void setBoundingBox(const double xMin, const double xMax, const double yMin, const double yMax,
                        const unsigned int nPoints = 120, const double offset = 0.0/1000);

    bool getMaxVariancePose(Vector &maxVariancePos);
    bool getMaxEstimatePos(Vector &maxEstimatePos);
    bool getNextSamplingPosition(Vector &nextSamplingPosition, bool nextRow = false);
    bool getNextRefinementPosition(Vector &nextSamplingPosition);
    bool getNextValidationPosition(Vector &validationPosition);
    void enableRefinement(){_refinementEnabled = true; _nextRefinementIndex = _paddingPoints;}
    void disableRefinement(){_refinementEnabled = false;}
    void enableValidation(){_validationEnable = true; _validationIndex = _paddingPoints;}
    void disableValidation(){_validationEnable = false;}
    bool validatePosition(Vector &validationPosition);

private:

    bool init(ResourceFinder& rf);
    double readOption(const string& main, const string& sub,  gurls::GurlsOptionsList *opt);
    gMat2D<double>* eval(const gMat2D<double> &X, gMat2D<double> &vars, gurls::GurlsOptionsList *opt);
    bool getMaxVariancePose(const gMat2D <double> &positions, gMat2D <double> &variance,
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
    gMat2D < double > _outputTesting;
    std::vector <double> _xPoints;
    std::vector <double> _yPoints;
    std::vector <double> _zPoints;
    int _paddingPoints;
    bool _repeatVar;
    unsigned long _nextSamplingIndex;
    unsigned long _nextRefinementIndex;
    unsigned long _maxRefinementIndex;

    unsigned long _validationIndex;
    bool _validationEnable;

    bool _refinementEnabled;
    double _maxX, _maxY, _minX, _minY;
    int _currentRow;
    int _currentCol;

    int _dummyIndex;

}; // end of class
} // end of namespace objectExploration
