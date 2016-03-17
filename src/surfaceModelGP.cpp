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


//void SurfaceModelGP::clearModel()
//{
//_objectModel.
//}

SurfaceModelGP::SurfaceModelGP(const std::string objectName)
{
    _dbgtg = "surfaceModelGP ( " + objectName + " ):";
    _objectName = objectName;
    _opt = NULL;
    _isValidModel = false;
    _isValidMaxVar = false;
    _inputTraining.resize(0,2);
    _paddingPoints = 0;
    _maxX = _minX = _maxY = _minY = 0;
    _repeatVar = false;
    _nextSamplingIndex = 0;
    _dummyIndex = 0;

 _currentRow = 4;
_currentCol = 1;


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


    _paddingPoints = _inputTraining.rows();
    // Updtate the internal vectors
    for(int i = 0; i < _inputTraining.rows(); i++)
    {
        _xPoints.push_back(_inputTraining[i].at(0));
        _yPoints.push_back(_inputTraining[i].at(1));
        _zPoints.push_back(_outputTraining[i].at(0));
    }

    //setBoundingBox();

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
    if(posXY.at(0) <= _minX || posXY.at(0) >= _maxX || posXY.at(1) <= _minY || posXY.at(1) >= _maxY)
    {
        cout << "Contact outside the bounding box, skipping it!" << endl;
        return;
    }
    // Add the new point to the matrix
    cout << "Adding a new contact point: " << _inputTraining.rows() - _paddingPoints + 1 << endl;
    //cout << posXY.at(0) << ", " << posXY.at(1) << " " << posZ.at(0) << endl;
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
    nholdouts->setValue(1);
    if(_opt->hasOpt("nholdouts"))
        _opt->removeOpt("nholdouts");
    _opt->addOpt("nholdouts", nholdouts);

    OptNumber *hoproportion = new OptNumber;
    hoproportion->setValue(0.1);
    if(_opt->hasOpt("hoproportion"))
        _opt->removeOpt("hoproportion");
    _opt->addOpt("hoproportion", hoproportion);

    OptNumber *epochs = new OptNumber;
    epochs->setValue(100000);
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

    //cout << "Training the model with size: " << _inputTraining.getSize() << endl;
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

void SurfaceModelGP::padBoundingBox()
{
    // padd the bounding box

    //Get the minmum and maximums

    gVec<double> *inputMax = _inputTraining.max(COLUMNWISE);
    gVec<double> *inputMin = _inputTraining.min(COLUMNWISE);
    gVec<double> *targetMin = _outputTraining.min(COLUMNWISE);

    double xMin, xMax, yMin, yMax, zMin;

    xMin = inputMin->at(0);
    xMax = inputMax->at(0);
    yMin = inputMin->at(1);
    yMax = inputMax->at(1);
    zMin = targetMin->at(0);



    padBoundingBox(xMin, xMax, yMin, yMax, zMin, 5, 0/1000);


    /*    cout << "xMin:" << xMin << " xMax: " << yMax << " yMin: " << yMin << " yMax: " << yMax << endl;
    cout << "Target Min: " << zMin << endl;

    double xSteps = (xMax - xMin)/5;
    double ySteps = (yMax - yMin)/5;

    double yValue = yMin;
    while (yValue <= yMax )
    {
        _xPoints.push_back(xMin);
        _yPoints.push_back(yValue);
        _zPoints.push_back(zMin);
        yValue += ySteps;
    }

    yValue = yMin + ySteps;
    while (yValue <= yMax )
    {
        _xPoints.push_back(xMax);
        _yPoints.push_back(yValue);
        _zPoints.push_back(zMin);
        yValue += ySteps;
    }

    double xValue = xMin;
    while(xValue <= xMax)
    {
        _xPoints.push_back(xValue);
        _yPoints.push_back(yMin);
        _zPoints.push_back(zMin);
        xValue += xSteps;
    }

    xValue = xMin + xSteps;
    while(xValue <= xMax)
    {
        _xPoints.push_back(xValue);
        _yPoints.push_back(yMax);
        _zPoints.push_back(zMin);
        xValue += xSteps;
    }


    // Update the gvectors
    _inputTraining.resize(_xPoints.size(),2);
    _outputTraining.resize(_zPoints.size(),1);


    for(int i =0; i < _xPoints.size(); i++ )
    {
        _inputTraining(i, 1) =  _yPoints.at(i);
        _inputTraining(i, 0) = _xPoints.at(i);
        _outputTraining(i,0) = _zPoints.at(i);
    }

*/
}

void SurfaceModelGP::padBoundingBox(double xMin, double xMax, double yMin, double yMax, double zMin, int nSteps, double offset)
{
    cout << "xMin:" << xMin << " xMax: " << xMax  << " yMin: " << yMin << " yMax: " << yMax << endl;
    cout << "Target Min: " << zMin << endl;

    xMin += offset;
    xMax -= offset;
    yMin += offset;
    yMax -= offset;

    assert(xMax > xMin);
    assert(yMax > yMin);

    double xSteps = (xMax - xMin)/nSteps;
    double ySteps = (yMax - yMin)/nSteps;

    //cout << "xSteps: " << xSteps << endl;
    //cout << "ySteps: " << ySteps << endl;

    double yValue = yMin;
    while (yValue <= yMax )
    {
        _xPoints.push_back(xMin);
        _yPoints.push_back(yValue);
        _zPoints.push_back(zMin);
        yValue += ySteps;
        _paddingPoints ++;
    }

    // Check if the last point added is at maximum
    // Othrewise add at maximum
    if(fabs(yValue - yMax) < fabs(ySteps))
    {
        _xPoints.push_back(xMin);
        _yPoints.push_back(yValue);
        _zPoints.push_back(zMin);
        _paddingPoints ++;
    }

    yValue = yMin;
    while (yValue <= yMax )
    {
        _xPoints.push_back(xMax);
        _yPoints.push_back(yValue);
        _zPoints.push_back(zMin);
        yValue += ySteps;
        _paddingPoints ++;
    }

    // Check if the last point added is at maximum
    // Othrewise add at maximum
    if(fabs(yValue - yMax) < fabs(ySteps))
    {
        _xPoints.push_back(xMax);
        _yPoints.push_back(yValue);
        _zPoints.push_back(zMin);
        _paddingPoints ++;
    }

    double xValue = xMin + xSteps;
    while(xValue < xMax)
    {
        _xPoints.push_back(xValue);
        _yPoints.push_back(yMin);
        _zPoints.push_back(zMin);
        xValue += xSteps;
        _paddingPoints ++;
    }

    xValue = xMin + xSteps;
    while(xValue < xMax)
    {
        _xPoints.push_back(xValue);
        _yPoints.push_back(yMax);
        _zPoints.push_back(zMin);
        xValue += xSteps;
        _paddingPoints ++;
    }


    //cout << "Here" << endl;
    // Update the gvectors
    _inputTraining.resize(_xPoints.size(),2);
    _outputTraining.resize(_zPoints.size(),1);


    for(int i =0; i < _xPoints.size(); i++ )
    {
        _inputTraining(i, 1) =  _yPoints.at(i);
        _inputTraining(i, 0) = _xPoints.at(i);
        _outputTraining(i,0) = _zPoints.at(i);
    }
    //cout << "Here 2" << endl;
}

void SurfaceModelGP::setBoundingBox(const double xMin, const double xMax, const double yMin, const double yMax,
                                    const unsigned int nPoints, const double offset)
{

    double *xlin = NULL;
    double *ylin = NULL;
    _maxX = xMax - offset;
    _maxY = yMax - offset;
    _minX = xMin + offset;
    _minY = yMin + offset;

    xlin = new (std::nothrow) double[nPoints];
    ylin = new (std::nothrow) double[nPoints];

    if(xlin == NULL || ylin == NULL)
    {
        cerr << _dbgtg << "could not allocate memory.";
        return;
    }


    linspace(xMin + offset, xMax - offset, nPoints, xlin);
    linspace(yMin + offset, yMax - offset, nPoints, ylin);

    cout << "Xmin: " << xMin << " XMax: " << xMax << endl;
    cout << "Ymin: " << yMin << " YMax: " << yMax << endl;


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

void SurfaceModelGP::setBoundingBox(const unsigned int nPoints, const double offset)
{
    // Generating training data

    gVec<double> *inputMax = _inputTraining.max(COLUMNWISE);
    gVec<double> *inputMin = _inputTraining.min(COLUMNWISE);


    setBoundingBox(inputMin->at(0), inputMax->at(0),
                   inputMin->at(1), inputMax->at(1),
                   nPoints, offset);

}

bool SurfaceModelGP::updateSurfaceEstimate(const unsigned int nPoints, const double offset)
{


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


    _outputTesting.resize(means->rows(),means->cols());
    _outputTesting = *means;

    //yarp::sig::Vector maxVariancePos;

    while(true){
        getMaxVariancePose(_inputTesting, vars, *means, _maxVariancePos);
        if(!_repeatVar)
        {

            break;
        }



    }


    _outputTesting.saveCSV("test.csv");
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




}


bool SurfaceModelGP::getMaxVariancePose(yarp::sig::Vector &maxVariancePos)
{

    maxVariancePos = _maxVariancePos;
    return _isValidMaxVar;




}

bool SurfaceModelGP::getNextSamplingPosition(yarp::sig::Vector &nextSamplingPosition, bool nextRow)
{


    ////// Hack to get surface data
    if(nextRow)
    {
        _currentRow--;
        _currentCol = 0;
        _nextSamplingIndex = _currentRow * 8;
    }

    _currentCol++;


    if(_currentCol > 5){
        if((--_currentRow) < 2)
            return false;
        else
            _currentCol = 1;

    }

    _nextSamplingIndex = _currentRow * 8 + _currentCol;
    cout << "Next:" << _nextSamplingIndex << endl;



    nextSamplingPosition.resize(3);
    nextSamplingPosition.zero();

    nextSamplingPosition[0] = _inputTesting(_nextSamplingIndex, 0);
    nextSamplingPosition[1] = _inputTesting(_nextSamplingIndex, 1);
    nextSamplingPosition[2] = _outputTesting(_nextSamplingIndex,0);

    std::ofstream myFile;
    myFile.open( (_objectName + "_model_nextPoint.csv").c_str());
    myFile << nextSamplingPosition.toString(10);
    //myFile << nextSamplingPosition[2] << endl;
    myFile.flush();
    myFile.close();


    return true;


    // Next sampling point is
    /*

    if(_dummyIndex > 7)
    {
        return false;
    }

    _nextSamplingIndex = _table[_dummyIndex][0] * 10 + _table[_dummyIndex][1];
    cout << "Next:" << _nextSamplingIndex << endl;
    _dummyIndex++;

    nextSamplingPosition.resize(3);
    nextSamplingPosition.zero();

    nextSamplingPosition[0] = _inputTesting(_nextSamplingIndex, 0);
    nextSamplingPosition[1] = _inputTesting(_nextSamplingIndex, 1);
    nextSamplingPosition[2] = _outputTesting(_nextSamplingIndex,0);

    std::ofstream myFile;
    myFile.open( (_objectName + "_model_nextPoint.csv").c_str());
    myFile << nextSamplingPosition.toString(10);
    //myFile << nextSamplingPosition[2] << endl;
    myFile.flush();
    myFile.close();


    return true;
    ////////

    */
    nextSamplingPosition.resize(3);
    nextSamplingPosition.zero();
    if(_nextSamplingIndex == 0)
        getMaxEstimatePos(nextSamplingPosition);

    if(_inputTesting(_nextSamplingIndex, 1) <= _minY || _inputTesting(_nextSamplingIndex, 1) >= _maxY )
        return false;

    nextSamplingPosition[0] = _inputTesting(_nextSamplingIndex, 0);
    nextSamplingPosition[1] = _inputTesting(_nextSamplingIndex, 1);
    nextSamplingPosition[2] = _outputTesting(_nextSamplingIndex,0);

    //std::ofstream myFile;
    myFile.open( (_objectName + "_model_nextPoint.csv").c_str());
    myFile << nextSamplingPosition.toString(10);
    //myFile << nextSamplingPosition[2] << endl;
    myFile.flush();
    myFile.close();

    _nextSamplingIndex++;
    return true;
}

bool SurfaceModelGP::getMaxEstimatePos(yarp::sig::Vector &maxEstimatePos)
{


    //unsigned long maxIndex;

    maxEstimatePos.resize(3);
    _nextSamplingIndex = static_cast<unsigned long>( _outputTesting.argmax(COLUMNWISE)->at(0));

    maxEstimatePos[0] = _inputTesting(_nextSamplingIndex, 0);
    maxEstimatePos[1] = _inputTesting(_nextSamplingIndex, 1);
    maxEstimatePos[2] = _outputTesting(_nextSamplingIndex,0);


    std::ofstream myFile;
    myFile.open( (_objectName + "_model_nextPoint.csv").c_str());
    myFile << maxEstimatePos.toString(10) << endl;
    myFile.flush();
    myFile.close();
    return true;

    //getMaxEstimatePos(_inputTesting, *vars)
}



bool SurfaceModelGP::getMaxVariancePose(const gMat2D<double> &positions,
                                        gMat2D<double> &variances,
                                        const gMat2D<double> &means,
                                        yarp::sig::Vector &maxVariancePos)
{
    _repeatVar = false;
    //yarp::sig::Vector maxVariancePos;
    maxVariancePos.clear();
    maxVariancePos.resize(3); // x, y  and z
    maxVariancePos.zero();

    unsigned long maxVarianceIndex = 0;
    double minVariance;
    double maxVariance;

    if(variances.getSize() > 0)
    {
        maxVariance = variances.max(gurls::COLUMNWISE)->at(0);
        //maxVarianceIndex = variances.argmax(gurls::COLUMNWISE)->at(0);
        minVariance = variances.min(gurls::COLUMNWISE)->at(0);

    }

    // cout << "Argmax:  " << maxVarianceIndex << endl;
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

    double epsilon = 5.0/1000;
    // Check if it is in the corners
    if( fabs(_maxX - maxVariancePos[0]) < epsilon){
        //cout << "At the edge" << endl;
        variances(maxVarianceIndex, 0) = minVariance;
        _repeatVar = true;
        return false;

    }

    if(fabs(_minX - maxVariancePos[0]) < epsilon){
        //cout << "At the edge" << endl;
        variances(maxVarianceIndex, 0) = minVariance;
        _repeatVar = true;
        return false;
    }

    if( fabs(_maxY - maxVariancePos[1]) < epsilon){
        //cout << "At the edge" << endl;
        variances(maxVarianceIndex, 0) = minVariance;
        _repeatVar = true;
        return false;
    }

    if(fabs(_minY - maxVariancePos[1]) < epsilon){
        //cout << "At the edge" << endl;
        variances(maxVarianceIndex, 0) = minVariance;
        _repeatVar = true;
        return false;
    }


    //cout << "Max var location ("<< maxVariancePos.toString() << ")" << endl;

    _isValidMaxVar = true;

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
