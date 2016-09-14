#pragma once
#include <surfaceModelGP.h>
#include <gurls++/gmat2d.h>
#include <yarp/os/Thread.h>

namespace objectExploration {

class TrainModelGPRegressionThread;
class TrainModelGPClassificationThread;

class SurfaceModelGPActive: public SurfaceModelGP{
public:
    SurfaceModelGPActive(const std::string objectName);
    ~SurfaceModelGPActive();
    virtual bool updateModel(); //This seems to be useless
    virtual bool trainModel(); // This the one run
    virtual bool updateSurfaceEstimate(const unsigned int nPoints, const double offset);

private:
    gMat2D<double>* evalClassification(const gMat2D<double> &X, gMat2D<double> &vars, gMat2D<double> &maxProbSurface, gurls::GurlsOptionsList  *opt);
    void getSurfaceUncertainty(const gMat2D<double> &classProb, gMat2D<double> &vars) const;
    void getMaxProbSurface(const gMat2D<double> &classProb, gMat2D<double> &maxProbSurface) const;
    void updateNBins();
    void binContacts();
    void configureRegressionOpt(gurls::GurlsOptionsList  *opt);
    void configureClassificationOpt(gurls::GurlsOptionsList  *opt);
    void normalise(gMat2D<double> &vector);

  //  bool trainGPClassificationModel();
  //  bool trainGPRegressionModel();

private:
    std::string _dbgtg;
    unsigned int _nBins;
    unsigned int _startBin;
    unsigned int _firstBinThreshold;
    double _lRate;
    gurls::GurlsOptionsList *_optClassification;

    gMat2D<double> _outputTrainingClassification;
    //gMat2D<double> _inputTrainingClassification;

    TrainModelGPRegressionThread *_GPRegressionThread;
    TrainModelGPClassificationThread *_GPClassificationThread;

};


class TrainModelGPRegressionThread: public yarp::os::Thread{
public:
    TrainModelGPRegressionThread(gMat2D<double> *inputTraining, gMat2D<double> *outputTraining,
    gurls::GurlsOptionsList *opt);
    void run();
    bool threadInit();
private:
    string _dbgtg;
    gMat2D<double> *_inputTraining;
    gMat2D<double> *_outputTraining;
    gurls::GurlsOptionsList *_opt;
    gurls::GURLS _objectModel;
};

class TrainModelGPClassificationThread: public yarp::os::Thread{
public:
    TrainModelGPClassificationThread(gMat2D<double> *inputTraining, gMat2D<double> *outputTraining,
    gurls::GurlsOptionsList *opt);
    void run();
    bool threadInit();
private:
    string _dbgtg;
    gMat2D<double> *_inputTraining;
    gMat2D<double> *_outputTraining;
    gurls::GurlsOptionsList *_opt;
    gurls::GURLS _objectModel;
};
} // end of namespace
