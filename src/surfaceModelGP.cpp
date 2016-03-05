#include "surfaceModelGP.h"
#include <gurls++/gvec.h>
#include <cmath>
#include <fstream>


namespace objectExploration
{

using std::endl;
using std::cerr;
using std::cout;
using namespace gurls;
using std::string;



SurfaceModelGP::SurfaceModelGP(const std::string objectName)
{
    _dbgtg = "surfaceModelGP ( " + objectName + " ):";
    _objectName = objectName;
    _opt = NULL;
    _isValidModel = false;
    _isValidMaxVar = false;
    _inputTraining.resize(0,2);

}


SurfaceModelGP::~SurfaceModelGP()
{
    if(_opt != NULL )
        delete _opt;
}



void SurfaceModelGP::loadContactData(const std::string type)
{

    string inputTrainingFileName =  _objectName + "_training_input_"  + type + ".csv";
    string outputTrainingFileName = _objectName + "_training_target_" + type + ".csv";;
    _inputTraining.readCSV(inputTrainingFileName);
    _outputTraining.readCSV(outputTrainingFileName);

    setBoundingBox();

}

void SurfaceModelGP::addContactPoint(const Vector fingertipPosition)
{
    gVec<double> posXY;
    gVec<double> posZ;


    posXY.resize(2);
    posZ.resize(1);
    posXY.at(0) = fingertipPosition[0];
    posXY.at(1) = fingertipPosition[1];
    posZ.at(0) = fingertipPosition[2];

    addContactPoint(posXY, posZ);

}

void SurfaceModelGP::addContactPoint(gVec<double> posXY, gVec<double> posZ)
{
    // Add the new point to the matrix
    cout << "Adding a new contact point: " << _inputTraining.rows() << ", " << _inputTraining.cols() << endl;
    cout << posXY.at(0) << ", " << posXY.at(1) << " " << posZ.at(0) << endl;
    //cout << "Size: " << posXY.getSize() << endl;
    //cout << "Data: " << *(posXY.getData()) << "," << *(posXY.getData()+1) << endl;

    _inputTraining.resize(_inputTraining.rows() + 1, 2);
    _outputTraining.resize(_outputTraining.rows() + 1, 1);//_outputTraining.cols());
    _inputTraining.setRow(posXY, _inputTraining.rows()-1);
    _outputTraining.setRow(posZ, _outputTraining.rows()-1);
     //_inputTraining[_inputTraining.rows() -1].at(0) = posXY.at(0);
     // _inputTraining[_inputTraining.rows() -1].at(0) = posXY.at(1);
    //cout << "T: " << _inputTraining[_inputTraining.rows() -1].at(0) << ", "
    //     << _inputTraining[_inputTraining.rows() -1].at(1) << endl;

    _xPoints.push_back(posXY.at(0));
    _yPoints.push_back(posXY.at(1));
    _zPoints.push_back(posZ.at(0));

    printTrainingData();


}

void SurfaceModelGP::saveContactPoints()
{

    string inputFileName = _objectName + "_training_input_GP";
    string outputFilename = _objectName + "_training_target_GP";
    _inputTraining.saveCSV(inputFileName + ".csv");
    _inputTraining.save(inputFileName + ".bin");

    _outputTraining.saveCSV(outputFilename + ".csv");
    _outputTraining.save(outputFilename + ".bin");
}

bool SurfaceModelGP::updateModel()
{
    //TODO: in future I can implement incremental update here
    return trainModel();
}

// Assumes that the contact data has already been loaded
bool SurfaceModelGP::trainModel(){

    bool ret = true;
    if(_inputTraining.getSize() == 0){
        cerr << _dbgtg << "no contact data is available. Cannot train the model." << endl;
        return false;
    }


    // specify the task sequence
    OptTaskSequence *seq = new OptTaskSequence();
    seq->addTask("split:ho");
    seq->addTask("paramsel:siglamhogpregr");
    seq->addTask("kernel:rbf");
    seq->addTask("optimizer:rlsgpregr");
    seq->addTask("predkernel:traintest");
    seq->addTask("pred:gpregr");

    //*seq << "split:ho" << "paramsel:siglamhogpregr" << "kernel:rbf"
    //     << "optimizer:rlsgpregr" << "predkernel:traintest" << "pred:gpregr";

    GurlsOptionsList * process = new GurlsOptionsList("processes", false);

    // defines instructions for training process
    OptProcess* process1 = new OptProcess();
    *process1 << GURLS::computeNsave
              << GURLS::computeNsave
              << GURLS::computeNsave
              << GURLS::computeNsave
              << GURLS::ignore
              << GURLS::ignore;
    process->addOpt("train", process1);

    // defines instructions for testing process
    OptProcess* process2 = new OptProcess();
    *process2 << GURLS::load
              << GURLS::load
              << GURLS::load
              << GURLS::load
              << GURLS::computeNsave
              << GURLS::computeNsave;
    process->addOpt("eval", process2);



    // build an options' structure
    string modelFileName = "GURLSgpr";
    _opt = new GurlsOptionsList(modelFileName, true);
    _opt->addOpt("seq", seq);
    _opt->addOpt("processes", process);

    OptNumber *nholdouts = new OptNumber;
    nholdouts->setValue(4);
    if(_opt->hasOpt("nholdouts"))
        _opt->removeOpt("nholdouts");
    _opt->addOpt("nholdouts", nholdouts);

    OptNumber *hoproportion = new OptNumber;
    hoproportion->setValue(0.1);
    if(_opt->hasOpt("hoproportion"))
        _opt->removeOpt("hoproportion");
    _opt->addOpt("hoproportion", hoproportion);

    OptNumber *epochs = new OptNumber;
    epochs->setValue(1000);
    if(_opt->hasOpt("epochs"))
        _opt->removeOpt("epochs");
    _opt->addOpt("epochs", epochs);

    OptString *hoperf = new OptString;
    hoperf->setValue("abserr");
    if(_opt->hasOpt("hoperf"))
        _opt->removeOpt("hoperf");

    _opt->addOpt("hoperf", hoperf);

    OptString *perfeval = new OptString;
    perfeval->setValue("abserr");
    if(_opt->hasOpt("perfeval"))
        _opt->removeOpt("perfeval");
    _opt->addOpt("perfeval", perfeval);

    //cout << "hoperf: " << opt->getOptAsString("hoperf") << endl;

    //cout << "Before: " << endl << opt->toString();


    string jobId0("train");

    cout << "Training the model with size: " << _inputTraining.getSize() << endl;
    // run gurls for training
    _objectModel.run(_inputTraining, _outputTraining, *_opt, jobId0);


    //opt->save(opt->getName());
    //cout << "After: " << endl << opt->toString();
    return ret;
}

void SurfaceModelGP::printTrainingData()
{

    //_inputTraining =  gMat2D<double>::zeros(_inputTesting.rows(), 2);
    for(int i =0; i < _inputTraining.rows(); i++ )
    {
        _inputTraining(i, 1) =  _yPoints.at(i);
        //cout << "G: " << _inputTraining[i].at(0) << ", "   << _inputTraining[i].at(1) << ", "  << _outputTraining[i].at(0) << endl;

        //cout << "V: " << _xPoints.at(i)  << ", " << _yPoints.at(i)  << ", " << _zPoints.at(i) << endl;
    }


}

void SurfaceModelGP::setBoundingBox(const unsigned int nPoints, const double offset)
{
    // Generating training data

    //unsigned int nPoints = 120;
    //double offset = 5.0/1000;

    gVec<double> *inputMax = _inputTraining.max(COLUMNWISE);
    gVec<double> *inputMin = _inputTraining.min(COLUMNWISE);

    //cout << "Max: " << inputMax->at(0) << ", " << inputMax->at(1) << endl;
    //cout << "Min: " << inputMin->at(0) << ", " << inputMin->at(1) << endl;


    double *xlin = NULL;
    double *ylin = NULL;
    xlin = new (std::nothrow) double[nPoints];
    ylin = new (std::nothrow) double[nPoints];

    if(xlin == NULL || ylin == NULL)
    {
        cerr << _dbgtg << "could not allocate memory.";
        return;
    }

    linspace(inputMin->at(0) + offset, inputMax->at(0) - offset, nPoints, xlin);
    linspace(inputMin->at(1) + offset, inputMax->at(1) - offset, nPoints, ylin);

    cout << "Xmin: " << inputMin->at(0) << " XMax: " << inputMax->at(0) << endl;
    cout << "Ymin: " << inputMin->at(1) << " YMax: " << inputMax->at(1) << endl;


    _inputTesting.resize(nPoints * nPoints, 2);




    for ( int x = 0; x < nPoints; x++ )
    {
        gVec < double > rowVect;
        rowVect.resize(2);
        rowVect.at(0) = xlin[x];
        for (int y = 0; y < nPoints; y++ )
        {
            rowVect.at(1) = ylin[y];
            _inputTesting.setRow(rowVect, nPoints * x + y);
        }
    }

    delete [] xlin;
    delete [] ylin;
}

bool SurfaceModelGP::updateSurfaceEstimate(const unsigned int nPoints, const double offset)
{


    // Generating training data

    //unsigned int nPoints = 120;
    //double offset = 5.0/1000;

    /*  gVec<double> *inputMax = _inputTraining.max(COLUMNWISE);
    gVec<double> *inputMin = _inputTraining.min(COLUMNWISE);

    //cout << "Max: " << inputMax->at(0) << ", " << inputMax->at(1) << endl;
    //cout << "Min: " << inputMin->at(0) << ", " << inputMin->at(1) << endl;


    double *xlin = NULL;
    double *ylin = NULL;
    xlin = new (std::nothrow) double[nPoints];
    ylin = new (std::nothrow) double[nPoints];

    if(xlin == NULL || ylin == NULL)
    {
        cerr << _dbgtg << "could not allocate memory.";
        return false;
    }

    linspace(inputMin->at(0) + offset, inputMax->at(0) - offset, nPoints, xlin);
    linspace(inputMin->at(1) + offset, inputMax->at(1) - offset, nPoints, ylin);

    cout << "Xmin: " << inputMin->at(0) << " XMax: " << inputMax->at(0) << endl;
    cout << "Ymin: " << inputMin->at(1) << " YMax: " << inputMax->at(1) << endl;

    gMat2D < double > inputTesting;
    inputTesting.resize(nPoints * nPoints, 2);




    for ( int x = 0; x < nPoints; x++ )
    {
        gVec < double > rowVect;
        rowVect.resize(2);
        rowVect.at(0) = xlin[x];
        for (int y = 0; y < nPoints; y++ )
        {
            rowVect.at(1) = ylin[y];
            inputTesting.setRow(rowVect, nPoints * x + y);
        }
    }

    // Use the test array as input to the gp process
*/


    gMat2D<double> vars;
    gMat2D<double>* means;
    if(_inputTesting.getSize() != 0)
    {

        means = this->eval(_inputTesting, vars, _opt);
    }
    else
    {
        cerr << _dbgtg << "No input testing, you forgot to initialise the bounding box for GP." << endl;
    }



    //cout << "Max: " <<  vars.max(gurls::COLUMNWISE)->at(0) << endl;


    //yarp::sig::Vector maxVariancePos;
    getMaxVariancePose(_inputTesting, vars, *means, _maxVariancePos);
    _isValidMaxVar = true;

    // Save the data for matlab visualisation
    //_inputTraining.saveCSV("newTraining.csv");
    _inputTesting.saveCSV(_objectName + "_model_input.csv");
    means->saveCSV(_objectName + "_model_output_GP.csv");
    vars.saveCSV(_objectName + "_model_variance_GP.csv");

    saveContactPoints();

    std::ofstream maxVarFile;
    maxVarFile.open( (_objectName + "_model_nextPoint.csv").c_str());
    maxVarFile << _maxVariancePos.toString(10) << " ";
    maxVarFile << vars.max(gurls::COLUMNWISE)->at(0);
    maxVarFile.flush();
    maxVarFile.close();


 //   delete[] xlin;
 //   delete[] ylin;

}

//void SurfaceModelGP::saveModelOutput()
//{

//}

bool SurfaceModelGP::getMaxVariancePose(yarp::sig::Vector &maxVariancePos)
{

    maxVariancePos = _maxVariancePos;
    return _isValidMaxVar;



    /*

    ///////////////////////////////


    unsigned int nPoints = 120;
    double offset = 5.0/1000;

    gVec<double> *inputMax = _inputTraining.max(COLUMNWISE);
    gVec<double> *inputMin = _inputTraining.min(COLUMNWISE);


    double *xlin = NULL;
    double *ylin = NULL;
    xlin = new (std::nothrow) double[nPoints];
    ylin = new (std::nothrow) double[nPoints];

    if(xlin == NULL || ylin == NULL)
    {
        cerr << _dbgtg << "could not allocate memory.";
        return false;
    }

    linspace(inputMin->at(0) + offset, inputMax->at(0) - offset, nPoints, xlin);
    linspace(inputMin->at(1) + offset, inputMax->at(1) - offset, nPoints, ylin);

    gMat2D < double > inputTesting;
    inputTesting.resize(nPoints * nPoints, 2);




    for ( int x = 0; x < nPoints; x++ )
    {
        gVec < double > rowVect;
        rowVect.resize(2);
        rowVect.at(0) = xlin[x];
        for (int y = 0; y < nPoints; y++ )
        {
            rowVect.at(1) = ylin[y];
            inputTesting.setRow(rowVect, nPoints * x + y);
        }
    }

    // Use the test array as input to the gp process




    gMat2D<double> vars;
    gMat2D<double>* means = this->eval(inputTesting, vars, _opt);



    //cout << "Max: " <<  vars.max(gurls::COLUMNWISE)->at(0) << endl;


    getMaxVariancePose(inputTesting, vars, *means, maxVariancePos);


    delete[] xlin;
    delete[] ylin;

    ////////////////////

    */
}

bool SurfaceModelGP::getMaxVariancePose(const gMat2D<double> &positions,
                                        const gMat2D<double> &variances,
                                        const gMat2D<double> &means,
                                        yarp::sig::Vector &maxVariancePos)
{
    //yarp::sig::Vector maxVariancePos;
    maxVariancePos.clear();
    maxVariancePos.resize(3); // x, y  and z
    maxVariancePos.zero();

    unsigned long maxVarianceIndex = 0;
    double maxVariance;

    if(variances.getSize() > 0)
        maxVariance = variances.max(gurls::COLUMNWISE)->at(0);

    // Search for the index of the maxVariance
    for( const double *var = variances.begin(); var != variances.end(); var++)
    {
        if(*var == maxVariance)
            break;

        maxVarianceIndex++;
    }

    maxVariancePos[0] = positions(maxVarianceIndex,0);
    maxVariancePos[1] = positions(maxVarianceIndex,1);
    maxVariancePos[2] = means(maxVarianceIndex,0);

    cout << "Max var location ("<< maxVariancePos.toString() << ")" << endl;
    return true;

}

gMat2D<double>* SurfaceModelGP::eval(const gMat2D<double> &X, gMat2D<double> &vars, gurls::GurlsOptionsList  *opt){

    typedef double T;

    gMat2D<T> empty;
    PredGPRegr<T> predTask;

    PredKernelTrainTest<T> predkTrainTest;

    if(opt->hasOpt("predkernel"))
        opt->removeOpt("predkernel");

    opt->addOpt("predkernel", predkTrainTest.execute(X, empty,*opt));

    GurlsOptionsList *pred = predTask.execute(X, empty, *opt);


    //std::cout << pred->toString() << std::endl;
    OptMatrix<gMat2D<T> >* pmeans = pred->getOptAs<OptMatrix<gMat2D<T> > >("means");
    pmeans->detachValue();

    gMat2D<T> &predMeans = pmeans->getValue();
    gMat2D<T> &predVars = pred->getOptValue<OptMatrix<gMat2D<T> > >("vars");

    const unsigned long n = predMeans.rows();
    const unsigned long t = predMeans.cols();


    vars.resize(n, t);

    for(int i = 0; i < n; i++)
    {

        for (int j = 0; j < t; j++)
        {
            vars(i,j) = predVars(i,j);
        }
    }


    delete pred;

    return &predMeans;
}

}
