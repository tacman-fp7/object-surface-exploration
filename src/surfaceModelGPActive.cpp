#include <surfaceModelGPActive.h>
#include <gurls++/opttasksequence.h>
#include <gurls++/gvec.h>
#include <cmath>

namespace objectExploration {

using std::cout;
using std::cerr;
using std::endl;
using namespace gurls;



SurfaceModelGPActive::SurfaceModelGPActive(const std::string objectName):SurfaceModelGP(objectName){
    _dbgtg = "SurfaceModelGPActive: ";
    _nBins = 1;
    _startBin = 1;
    _firstBinThreshold = 15;
    _lRate = 0.025;

    string modelFileName = _objectName + "_GPClassification";
    _optClassification = new GurlsOptionsList(modelFileName, true);


    _GPClassificationThread = new TrainModelGPClassificationThread(&_inputTraining, &_outputTrainingClassification, _optClassification);
    _GPRegressionThread = new TrainModelGPRegressionThread(&_inputTraining, &_outputTraining, _opt);

    configureRegressionOpt(_opt);
    configureClassificationOpt(_optClassification);


}


SurfaceModelGPActive::~SurfaceModelGPActive(){
    if(_optClassification != NULL){
        delete _optClassification;
    }
}

void SurfaceModelGPActive::configureClassificationOpt(GurlsOptionsList *opt){

    // specify the task sequence
    OptTaskSequence *seq = new OptTaskSequence();
    seq->addTask("split:ho");
    seq->addTask("paramsel:siglamho");
    seq->addTask("kernel:rbf");
    seq->addTask("optimizer:rlsdual");
    seq->addTask("predkernel:traintest");
    seq->addTask("pred:dual");
    //seq->addTask("conf:precrec");
    //seq->addTask("perf:macroavg");
    seq->addTask("conf:boltzman");

    GurlsOptionsList * process = new GurlsOptionsList("processes", false);

    // defines instructions for training process
    OptProcess* process1 = new OptProcess();
    *process1 << GURLS::computeNsave
              << GURLS::computeNsave
              << GURLS::computeNsave
              << GURLS::computeNsave
              << GURLS::ignore
              << GURLS::ignore
              << GURLS::ignore;
    //<< GURLS::ignore;
    process->addOpt("train", process1);

    // defines instructions for testing process
    OptProcess* process2 = new OptProcess();
    *process2 << GURLS::load
              << GURLS::load
              << GURLS::load
              << GURLS::load
              << GURLS::computeNsave
              << GURLS::computeNsave
              << GURLS::computeNsave;
    //<< GURLS::computeNsave;
    process->addOpt("eval", process2);




    // build an options' structure
    // string modelFileName = "GURLSclassification";
    // _opt = new GurlsOptionsList(modelFileName, true);
    opt->addOpt("seq", seq);
    opt->addOpt("processes", process);

    OptNumber *nholdouts = new OptNumber;
    nholdouts->setValue(1);
    if(opt->hasOpt("nholdouts"))
        opt->removeOpt("nholdouts");
    opt->addOpt("nholdouts", nholdouts);

    OptNumber *hoproportion = new OptNumber;
    hoproportion->setValue(0.1);
    if(opt->hasOpt("hoproportion"))
        opt->removeOpt("hoproportion");
    opt->addOpt("hoproportion", hoproportion);

    OptNumber *epochs = new OptNumber;
    epochs->setValue(10000);
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



}

void SurfaceModelGPActive::configureRegressionOpt(GurlsOptionsList *opt){
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
    //string modelFileName = "GURLSgpr";
    //_opt = new GurlsOptionsList(modelFileName, true);
    opt->addOpt("seq", seq);
    opt->addOpt("processes", process);

    OptNumber *nholdouts = new OptNumber;
    nholdouts->setValue(2);
    if(opt->hasOpt("nholdouts"))
        opt->removeOpt("nholdouts");
    opt->addOpt("nholdouts", nholdouts);

    OptNumber *hoproportion = new OptNumber;
    hoproportion->setValue(0.1);
    if(opt->hasOpt("hoproportion"))
        opt->removeOpt("hoproportion");
    opt->addOpt("hoproportion", hoproportion);

    OptNumber *epochs = new OptNumber;
    epochs->setValue(100000);
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

}

bool SurfaceModelGPActive::updateModel(){
    return (trainModel());
}

bool SurfaceModelGPActive::trainModel(){


    // decide how many bins should we have
    updateNBins();

    unsigned long trainingPoints = _inputTraining.rows() - _paddingPoints.size();
    if(trainingPoints > 0){
        binContacts();
        // run sufrace classificatin
        // trainGPClassificationModel();
        _GPClassificationThread->start();
    }
    _GPRegressionThread->start();

    while(_GPClassificationThread->isRunning() || _GPRegressionThread->isRunning()){
        ;
    }


    // Now update the surface model


}

void SurfaceModelGPActive::updateNBins(){

    unsigned long trainingPoints = _inputTraining.rows() - _paddingPoints.size();

    if(trainingPoints > (_firstBinThreshold * 5)){
        _nBins = 12;
    } else if(trainingPoints > (_firstBinThreshold * 4)){
        _nBins = 9;
    } else if(trainingPoints > (_firstBinThreshold * 3)){
        _nBins = 6;
        _startBin = 3;
    } else if(trainingPoints > (_firstBinThreshold * 2)){
        _nBins = 3;
        _startBin = 2;
    } else if(trainingPoints > _firstBinThreshold * 1){
        _nBins = 1;
    }
}

void SurfaceModelGPActive::binContacts(){


    double binSize = (_outputTraining.max(gurls::COLUMNWISE)->at(0) - _outputTraining.min(gurls::COLUMNWISE)->at(0))/(_nBins + 1);

    //std::cout << _outputTraining.max(gurls::COLUMNWISE)->getSize() << ", " << _outputTraining.min(gurls::ROWWISE)->getSize() << std::endl;
    _outputTrainingClassification.resize(_outputTraining.rows(), _nBins);


    //_outputTrainingClassification.zeros(_outputTraining.rows(), _nBins);



    for(int i = 0; i < _outputTrainingClassification.rows(); i++){
        for(int j = 0; j < _outputTrainingClassification.cols(); j++){
            _outputTrainingClassification(i,j) = NEGATIVE_EXAMPLE;
        }
    }


    for (int i = 0; i < _outputTraining.rows(); i++){
        double height = std::fabs(_outputTraining(i,0) - _outputTraining.min(gurls::COLUMNWISE)->at(0));

        for (int j = _nBins -1; j >= 0; j--){
            if(height > binSize * (j + 1)){
                _outputTrainingClassification(i,j) = POSITIVE_EXAMPLE;
                break;
            }
        }
    }

    _outputTrainingClassification.saveCSV("trainingClassification.csv");
    // I may have to make sure that each lass has at least one sample

}



/*void SurfaceModelGPActive::binContacts(){

    double minTarget;


    _outputTrainingClassification.resize(_inputTraining.rows() - _paddingPoints, 1);
    _inputTrainingClassification.resize(_inputTraining.rows() - _paddingPoints , _inputTraining.cols());
    for(int i = _paddingPoints; i < _inputTraining.rows(); i++){
        for(int j = 0; j < _inputTraining.cols(); j++){
            _inputTrainingClassification(i - _paddingPoints, j) = _inputTraining(i,j);
            _outputTrainingClassification(i - _paddingPoints, 0) = _outputTraining(i,0);
        }
    }



    minTarget = _outputTrainingClassification.min(gurls::COLUMNWISE)->at(0);
    double binSize = (_outputTrainingClassification.max(gurls::COLUMNWISE)->at(0) - _outputTrainingClassification.min(gurls::COLUMNWISE)->at(0))/(_nBins + 1);

    //std::cout << _outputTraining.max(gurls::COLUMNWISE)->getSize() << ", " << _outputTraining.min(gurls::ROWWISE)->getSize() << std::endl;
    _outputTrainingClassification.resize(_outputTrainingClassification.rows(), _nBins);
    //_outputTrainingClassification.zeros(_outputTraining.rows(), _nBins);



    for(int i = 0; i < _outputTrainingClassification.rows(); i++){
        for(int j = 0; j < _outputTrainingClassification.cols(); j++){
            _outputTrainingClassification(i,j) = 0;
        }
    }


    _outputTrainingClassification.saveCSV("trainingClassificationZeros.csv");

    for (int i = _paddingPoints; i < _outputTraining.rows(); i++){
        double height = std::fabs(_outputTraining(i,0) - minTarget);

        for (int j = 0; j < _nBins; j++){
            if(height < binSize * (j + 1)){
                _outputTrainingClassification(i - _paddingPoints,j) = 1;
                break;
            }
        }
    }

    _outputTrainingClassification.saveCSV("trainingClassification.csv");
    // I may have to make sure that each lass has at least one sample

}*/

// Assumes that the contact data has already been loaded
/*bool SurfaceModelGPActive::trainGPClassificationModel(){

    bool ret = true;
    if(_inputTraining.getSize() == 0){
        cerr << _dbgtg << "no contact data is available. Cannot train the model." << endl;
        return false;
    }


    // specify the task sequence
    OptTaskSequence *seq = new OptTaskSequence();
    seq->addTask("split:ho");
    seq->addTask("paramsel:siglamho");
    seq->addTask("kernel:rbf");
    seq->addTask("optimizer:rlsdual");
    seq->addTask("predkernel:traintest");
    seq->addTask("pred:dual");
    seq->addTask("perf:macroavg");
    seq->addTask("conf:bolzman");

    GurlsOptionsList * process = new GurlsOptionsList("processes", false);

    // defines instructions for training process
    OptProcess* process1 = new OptProcess();
    *process1 << GURLS::computeNsave
              << GURLS::computeNsave
              << GURLS::computeNsave
              << GURLS::computeNsave
              << GURLS::ignore
              << GURLS::ignore
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
              << GURLS::computeNsave
              << GURLS::computeNsave
              << GURLS::computeNsave;
    process->addOpt("eval", process2);



    // build an options' structure
    string modelFileName = "GURLSclassification";
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
    epochs->setValue(10000);
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


    string jobId0("train");

    //cout << "Training the model with size: " << _inputTraining.getSize() << endl;
    // run gurls for training
    _objectModel.run(_inputTraining, _outputTraining, *_opt, jobId0);


    //opt->save(opt->getName());
    //cout << "After: " << endl << opt->toString();
    return ret;
}*/


bool SurfaceModelGPActive::updateSurfaceEstimate(const unsigned int nPoints, const double offset){

    // Two things need to be done
    // 1) eval on the classification
    // 2) eval on the regression
    // Regression can always be run


    gMat2D<double> varsClassification;
    gMat2D<double>* meansClassification;
    gMat2D<double> surfaceGPClassification;

    gMat2D<double> varsRegression;
    gMat2D<double>* meansRegression;
    //gMat2D<double> *urgh;
    gMat2D<double> combinedVar;

    unsigned long trainingPoints;

    if(_inputTesting.getSize() != 0){

        // Run regression
        meansRegression = eval(_inputTesting, varsRegression, _opt);

        normalise(varsRegression);

        // If we have enough points, run classification
        trainingPoints = _inputTraining.rows() - _paddingPoints.size();
        if(trainingPoints > 0){

   /*         if(_classificationWraper != NULL){
                delete _classificationWraper;
            }
            _classificationWraper = new KernelRLSWrapper<double>("classWrapper");
            _classificationWraper->setKernelType(KernelRLSWrapper<double>::RBF);
            _classificationWraper->setProblemType(KernelRLSWrapper<double>::CLASSIFICATION);
            _classificationWraper->
            _classificationWraper->train(_inputTraining, _outputTrainingClassification);
            urgh =  _classificationWraper->eval(_inputTesting);

            urgh->saveCSV("wrapper.csv");
*/



/*
                    string jobId0("eval");

                    gMat2D<double> dummyTest;
                    //dummyTest.resize(_inputTesting.rows(), _outputTrainingClassification.cols());
                    //dummyTest.zeros(_inputTesting.rows(), _outputTrainingClassification.cols());
                   delete _optClassification;
                    _optClassification = new GurlsOptionsList("dummy", true);
                    configureClassificationOpt(_optClassification);

                    _objectModelTest.run(_inputTesting, dummyTest, *_optClassification, jobId0);

            //gurls::GurlsOption *classOpt = _optClassification->getOpt("pred");
        OptMatrix< gMat2D< double > >   a = _optClassification->getOptValue<  OptMatrix< gMat2D< double > > >("pred");

        a.getValue().saveCSV("omg.csv");
            //gurls::gMat2D<double> classOpt = _optClassification->getOptValue< gMat2D<double> >("pred");

            //cout << *classOpt << endl;
*/
            meansClassification = evalClassification(_inputTesting, varsClassification, surfaceGPClassification,  _optClassification);
            combinedVar.resize(varsRegression.rows(), varsRegression.cols());
            combinedVar = varsClassification * (1 - _lRate) + varsRegression * _lRate;
        }
        else{
            combinedVar.resize(varsRegression.rows(), varsRegression.cols());
            combinedVar = varsRegression;
            // combinedVar = varsClassification * (1 - _lRate) + varsRegression * _lRate;combinedVar = varsRegression;
        }
    }
    else{
        cerr << _dbgtg << "No input testing, you forgot to initialise the bounding box for GP." << endl;
    }

    //_outputTesting.resize(meansRegression->rows(),meansRegression->cols());
    //_outputTesting = *meansRegression;





    //surfaceUncertainty * (1 - this.lRate) + spatialUncertainty * this.lRate;
    while(true){
        getMaxVariancePose(_inputTesting, combinedVar, *meansRegression, _maxVariancePos);
        if(!_repeatVar){
            break;
        }
    }

    //_outputTesting.saveCSV("test.csv");
    // Save the data for matlab visualisation
    //_inputTraining.saveCSV("newTraining.csv");
    _inputTesting.saveCSV(_objectName + "_model_input.csv");
    meansRegression->saveCSV(_objectName + "_model_output_GPRegression.csv");
    varsRegression.saveCSV(_objectName + "_model_variance_GPRegression.csv");
    combinedVar.saveCSV(_objectName + "_model_variance_GPCombined.csv");
    if(trainingPoints > 0){
        meansClassification->saveCSV(_objectName + "_model_output_GPClassification.csv");
        surfaceGPClassification.saveCSV(_objectName + "_model_output_GPSurfaceClassification.csv");
        varsClassification.saveCSV(_objectName + "_model_variance_GPClassification.csv");

    }
    saveContactPoints();

    std::ofstream maxVarFile;
    maxVarFile.open( (_objectName + "_model_nextPoint.csv").c_str());
    maxVarFile << _maxVariancePos.toString(10) << " ";
    maxVarFile << varsRegression.max(gurls::COLUMNWISE)->at(0);
    maxVarFile.flush();
    maxVarFile.close();


}


gMat2D<double>* SurfaceModelGPActive::evalClassification(const gMat2D<double> &X, gMat2D<double> &vars, gMat2D<double> &maxProbSruface, gurls::GurlsOptionsList  *opt){

    typedef double T;

    gMat2D<T> empty; // dummy input for evaluation
    PredDual<T> predTask;

    PredKernelTrainTest<T> predkTrainTest;
    //std::cout << opt->toString() << std::endl;
    //if(opt->hasOpt("optimizer"))
    //    std::cout << "has optimizer here too" << endl;

    // Make sure the predkernel is empty.
    if(opt->hasOpt("predkernel")){
        opt->removeOpt("predkernel");
    }
    // Add a new prediction kernel
    opt->addOpt("predkernel", predkTrainTest.execute(X, empty,*opt));

    // Perfrom prediction

    OptMatrix< gMat2D< T > >  *classPredOptMatrix = predTask.execute(X, empty, *opt);


    // Get the class predition matrix
    //OptMatrix<gMat2D<T> >* classPredOptMatrix = pred->getOptAs<OptMatrix<gMat2D<T> > >("pred");
    //classPredOptMatrix->detachValue();

    // Extract the class prediction data
    gMat2D<T> &classPred = classPredOptMatrix->getValue();


    /*classPred.saveCSV("predBefore.csv");
    classPred -= (classPred.max(gurls::COLUMNWISE)->at(0) - classPred.min(gurls::COLUMNWISE)->at(0))/2;
classPred.saveCSV("predAfter.csv");*/
    // Conver the class preditions to uncertainty
    getSurfaceUncertainty(classPred, vars);

    getMaxProbSurface(classPred, maxProbSruface);

    //    delete pred;

    return &classPred;
}

void SurfaceModelGPActive::getMaxProbSurface(const gMat2D<double> &classProb, gMat2D<double> &maxProbSurface) const{


    maxProbSurface.resize(classProb.rows(), 1);
    // This zero out is not working
    maxProbSurface.zeros(classProb.rows(),1);

    for (int i = 0; i < classProb.rows(); i++){
        maxProbSurface(i,0) = 0.0;
    }
    //maxProbSurface.saveCSV("text.csv");
    if(classProb.cols() < 2){
        for (unsigned long i = 0; i < classProb.rows(); i++){
            if(classProb(i,0) > 0){
                maxProbSurface(i,0) = 1;
            }
        }
    }
    else{
        for (unsigned long i = 0; i < classProb.rows(); i++){
            double maxProb = classProb(i, 0);

            for (unsigned long j = 1; j < classProb.cols(); j++){
                if(classProb(i,j) > maxProb){
                    maxProbSurface(i,0) = j;
                    maxProb = classProb(i,j);
                }
            }
        }
    }
}


void SurfaceModelGPActive::getSurfaceUncertainty(const gMat2D<double> &classProb, gMat2D<double> &vars) const{

    // GURLS outputs class probabilities in nSamples by nClasses matrix. For each sample
    // A large positive number indicates it has strong evidence for the class
    // A large negative number otherwise.

    // As a first step we take absolute value of each value

    gMat2D<double> tempClassProb;

    unsigned long nSamples = classProb.rows();
    unsigned int nClasses = classProb.cols();
    //classProb.saveCSV("classProb.csv");
    tempClassProb.resize(nSamples, nClasses);

    for (unsigned long sample = 0; sample < nSamples; sample++){
        for(unsigned int mclass = 0; mclass < nClasses; mclass++){
            tempClassProb(sample, mclass) = std::fabs(classProb(sample, mclass));
        }
    }

    //tempClassProb.saveCSV("classProbAbs.csv");

    // Second step, normalise each precition and convert it to uncertainty
    gVec<double> *max = tempClassProb.max(gurls::COLUMNWISE);
    gVec<double> *min = tempClassProb.min(gurls::COLUMNWISE);

    for (unsigned long sample = 0; sample < nSamples; sample++){
        for(unsigned int mclass = 0; mclass < nClasses; mclass++){
            tempClassProb(sample, mclass) = 1 - ((tempClassProb(sample, mclass) - min->at(mclass)) /
                                                 (max->at(mclass) - min->at(mclass)));
        }
    }

   // tempClassProb.saveCSV("classProbAbsNorm.csv");

    // The third step is to combine all of them into one
    vars.resize(nSamples, 1); // Only one row
    vars.setColumn(*tempClassProb.max(gurls::ROWWISE), 0);

    //vars.saveCSV("classProbAbsNormMax.csv");

}

void SurfaceModelGPActive::normalise(gMat2D<double> &vector){

    gVec<double> *max = vector.max(gurls::COLUMNWISE);
    gVec<double> *min = vector.min(gurls::COLUMNWISE);

    for (unsigned long sample = 0; sample < vector.rows(); sample++){
        for(unsigned int mclass = 0; mclass < vector.cols(); mclass++){
            vector(sample, mclass) = (vector(sample, mclass) - min->at(mclass)) /
                                                 (max->at(mclass) - min->at(mclass));
        }
    }
}

void TrainModelGPRegressionThread::run(){

    if(_inputTraining->getSize() == 0){
        cerr << _dbgtg << "no contact data is available. Cannot train the model." << endl;
        return;// false;
    }




    string jobId0("train");

    if(_opt->hasOpt("kernel")){
        _opt->removeOpt("kernel");
    }
    //cout << "Training the model with size: " << _inputTraining.getSize() << endl;
    // run gurls for training
    //cout << "Input: " << _inputTraining->rows() << ", " << _inputTraining->cols() << endl;
    //cout << "Target " << _outputTraining->rows() << ", " << _outputTraining->cols() << endl;
    _objectModel.run(*_inputTraining, *_outputTraining, *_opt, jobId0);


    //opt->save(opt->getName());
    //cout << "After: " << endl << opt->toString();
    //return ret;

    //   std::cout << _opt->toString() << std::endl;
    //    if(_opt->hasOpt("optimizer"))
    //        std::cout << "has optimizer" << std::endl;
}

TrainModelGPRegressionThread::TrainModelGPRegressionThread(gMat2D<double> *inputTraining,
                                                           gMat2D<double> *outputTraining, GurlsOptionsList *opt){

    _dbgtg = "TrainModelGPRegressionThread: ";
    _inputTraining = inputTraining;
    _outputTraining = outputTraining;
    _opt = opt;

}

bool TrainModelGPRegressionThread::threadInit(){



    if(_inputTraining == NULL || _outputTraining == NULL){
        cerr << _dbgtg << "thread not initialised" << endl;
        return false;
    }

    return true;
}

void TrainModelGPClassificationThread::run(){

    bool ret = true;
    if(_inputTraining->getSize() == 0){
        cerr << _dbgtg << "no contact data is available. Cannot train the model." << endl;
        return;// false;
    }



    string jobId0("train");

    //cout << "Training the model with size: " << _inputTraining.getSize() << endl;
    // run gurls for training
    _objectModel.run(*_inputTraining, *_outputTraining, *_opt, jobId0);



}

TrainModelGPClassificationThread::TrainModelGPClassificationThread(gMat2D<double> *inputTraining,
                                                                   gMat2D<double> *outputTraining, GurlsOptionsList *opt){

    _dbgtg = "TrainModelGPClassificationThread: ";
    _inputTraining = inputTraining;
    _outputTraining = outputTraining;
    _opt = opt;

}

bool TrainModelGPClassificationThread::threadInit(){

    if(_inputTraining == NULL || _outputTraining == NULL){
        cerr << _dbgtg << "thread not initialised" << endl;
        return false;
    }

    return true;
}

} // end of namespace
