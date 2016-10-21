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
using std::vector;

struct point3d{

    point3d(double x, double y, double z):x(x), y(y), z(z){}
    double x;
    double y;
    double z;
};


typedef vector< point3d > fingertipData_t;
typedef fingertipData_t::iterator fingertipDataItr;
typedef vector< fingertipData_t> contactLocation_t;
typedef contactLocation_t::iterator contactLocationItr;

class SurfaceModel{
public:

    virtual bool trainModel() = 0;
    virtual bool updateModel() = 0;
    virtual bool updateSurfaceEstimate(const unsigned int nGrid = 120, const double offset = 0.0/1000) = 0;
    virtual void loadContactData(const std::string type) = 0;
    virtual void addContactPoint(const Vector fingertipPosition, const int fingerID=0) = 0;
    virtual void addContactPoint(gVec<double> posXY, gVec<double> posZ, const int fingerID) = 0;
    virtual void saveContactPoints() = 0;
    virtual void padBoundingBox() = 0;
    virtual void padBoundingBox(double xMin, double xMax, double yMin, double yMax, double zMin,
                                int nSteps = 5, double offset = 0.0/1000) = 0;
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
    virtual bool trainModel();
    virtual bool updateModel();
    virtual bool updateSurfaceEstimate(const unsigned int nGrid = 120, const double offset = 0.0/1000);
    void loadContactData(const std::string type);
    void addContactPoint(const Vector fingertipPosition, const int fingerID=0);
    void addContactPoint(gVec<double> posXY, gVec<double> posZ, const int fingerID=0);
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
    void enableRefinement(){_refinementEnabled = true; _nextRefinementIndex = _paddingPoints.size();}
    void disableRefinement(){_refinementEnabled = false;}
    void enableValidation(){_validationEnable = true; _validationIndex = _paddingPoints.size();}
    void disableValidation(){_validationEnable = false;}
    bool validatePosition(Vector &validationPosition);

protected:
    bool getMaxVariancePose(const gMat2D <double> &positions, gMat2D <double> &variance,
                            const gMat2D <double> &means, Vector &maxVariancePos);
    gMat2D<double>* eval(const gMat2D<double> &X, gMat2D<double> &vars, gurls::GurlsOptionsList *opt);


private:

    bool init(ResourceFinder& rf);
    double readOption(const string& main, const string& sub,  gurls::GurlsOptionsList *opt);


    void mergeFingerData();
    //void addPaddingPoints(double startPoint, double endPoint, double constAxis, double targetValue);
protected:
    std::string _objectName;
    gurls::GURLS _objectModel;
    gurls::GurlsOptionsList* _opt;
    gMat2D<double> _inputTraining;
    gMat2D<double> _outputTraining;
    gMat2D < double > _inputTesting;
    gMat2D < double > _outputTesting;
    Vector _maxVariancePos;
    bool _repeatVar;
    //int _nPaddingPoints;
    fingertipData_t _paddingPoints;


private:

    bool _isValidMaxVar;
    string _dbgtg;
    bool _isValidModel;




    contactLocation_t _contactLocations;


    //vector< vector <double> > _xPoints;
    //vector< vector <double> > _yPoints;
    //vector< vector <double> > _zPoints;


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
