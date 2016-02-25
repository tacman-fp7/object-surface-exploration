#include "surfacemodelgp.h"
#include <gurls++/gvec.h>
#include <cmath>


namespace objectExploration
{

using std::endl;
using std::cerr;
using std::cout;
using namespace gurls;
using std::string;



SurfaceModelGP::SurfaceModelGP()
{
    _dbgtg = "surfaceModelGP: ";
    opt = NULL;

}


SurfaceModelGP::~SurfaceModelGP()
{
    if(opt != NULL )
        delete opt;
}

void SurfaceModelGP::loadContactData()
{

    string inputTrainingFileName = "inputTraining.csv";
    string outputTrainingFileName = "outputTraining.csv";
    _inputTraining.readCSV(inputTrainingFileName);
    _outputTraining.readCSV(outputTrainingFileName);

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
    //seq->addTask("paramsel:siglamloogpregr");
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
    opt = new GurlsOptionsList(modelFileName, true);
    opt->addOpt("seq", seq);
    opt->addOpt("processes", process);

    OptNumber *nholdouts = new OptNumber;
    nholdouts->setValue(4);
    if(opt->hasOpt("nholdouts"))
        opt->removeOpt("nholdouts");
    opt->addOpt("nholdouts", nholdouts);

    OptNumber *hoproportion = new OptNumber;
    hoproportion->setValue(0.1);
    if(opt->hasOpt("hoproportion"))
        opt->removeOpt("hoproportion");
    opt->addOpt("hoproportion", hoproportion);

    OptNumber *epochs = new OptNumber;
    epochs->setValue(1000);
    if(opt->hasOpt("epochs"))
        opt->removeOpt("epochs");
    opt->addOpt("epochs", epochs);

    OptString *hoperf = new OptString;
    hoperf->setValue("abserr");
    if(opt->hasOpt("hoperf"))
        opt->removeOpt("hoperf");

    opt->addOpt("hoperf", hoperf);

    OptString *perfeval = new OptString;
    perfeval->setValue("abserr");
    if(opt->hasOpt("perfeval"))
        opt->removeOpt("perfeval");
    opt->addOpt("perfeval", perfeval);

    //cout << "hoperf: " << opt->getOptAsString("hoperf") << endl;

    //cout << "Before: " << endl << opt->toString();


    string jobId0("train");


    // run gurls for training
    _objectModel.run(_inputTraining, _outputTraining, *opt, jobId0);
    //opt->save(opt->getName());
    //cout << "After: " << endl << opt->toString();
    return ret;
}


bool SurfaceModelGP::saveMeshCSV()
{


    // Generating training data

    unsigned int nPoints = 120;
    double offset = 5;

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
    gMat2D<double>* means = this->eval(inputTesting, vars, opt);



    cout << "Max: " <<  vars.max(gurls::COLUMNWISE)->at(0) << endl;


    getMaxVariancePose(inputTesting, vars, *means);

    // Save the data for matlab visualisation
    inputTesting.saveCSV("inputTesting.csv");
    means->saveCSV("means.csv");
    vars.saveCSV("vars.csv");

    delete[] xlin;
    delete[] ylin;

}

yarp::sig::Vector SurfaceModelGP::getMaxVariancePose(const gMat2D<double> &positions,
                                                     const gMat2D<double> &variances,
                                                     const gMat2D<double> &means)
{
    yarp::sig::Vector maxVariancePos;
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

    cout << "Max var: " << variances(maxVarianceIndex, 0) << ","
         << maxVariancePos.toString() << endl;
    return maxVariancePos;

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
