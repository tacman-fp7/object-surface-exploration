#include "objectModelGrid.h"
#include <fstream>
#include <assert.h>
#include <string>
#include <iostream>


namespace objectExploration {
using std::string;
using std::cout;
using std::endl;

void ObjectModelGrid::addContactPoint(const yarp::sig::Vector fingertipPosition){

    assert(fingertipPosition.size() == 3);
    cout << "Contact position: " << fingertipPosition.toString() << endl;
    _contactPoints.push_back(fingertipPosition);


}


void ObjectModelGrid::saveContactPoints(){

    string inputFileName = _objectName + "_training_input_GP.csv";
    string outputFilename = _objectName + "_training_target_GP.csv";

    std::ofstream inputFile;
    std::ofstream outputFile;
    inputFile.open(inputFileName.c_str());
    outputFile.open(outputFilename.c_str());
    inputFile.precision(6);
    outputFile.precision(6);
    inputFile.setf( std::ios_base::fixed, std::ios_base::floatfield);
    outputFile.setf(std::ios_base::fixed, std::ios_base::floatfield);

    for (std::vector< yarp::sig::Vector>::iterator contactPoint = _contactPoints.begin(); contactPoint != _contactPoints.end(); contactPoint++){
        inputFile  << (*contactPoint)[0] << ", " << (*contactPoint)[1] << endl;
        outputFile << (*contactPoint)[2] << ", " << endl;
    }

    inputFile.close();
    outputFile.close();
}

bool ObjectModelGrid::getNextSamplingPos(yarp::sig::Vector& nextSamplingPoint){

    bool ret = false;
    nextSamplingPoint.resize(3);
    // Fix the x coordinate, move along the y
    // increment x when y is at or beyon max

    if(_nextSamplingPoint[1] < _yMax){
        _nextSamplingPoint[1] += _stepSize;
        ret = true;
    }
    else if(_nextSamplingPoint[0] > _xMin){
            _nextSamplingPoint[0] -= _stepSize;
            _nextSamplingPoint[1] = _yMin;
            ret = true;
    }
    else{
        ret = false;
    }

    nextSamplingPoint = _nextSamplingPoint;
    return ret;
}

void ObjectModelGrid::init(yarp::sig::Vector startingPos, yarp::sig::Vector endingPos){

    _stepSize; // Needs to be defined
    _xMax = startingPos[0];
    _xMin = _xMax - _searchSpaceWidth;// 90.0/1000; // should go in the config file


    if(startingPos[1] < endingPos[1])
    {
        _yMin = startingPos[1];
        _yMax = endingPos[1];
    }
    else
    {
        _yMin = endingPos[1];
        _yMax = startingPos[1];
    }


    //std::cout  << _xMin << ", " << _xMax << ", " << _yMin << ", " << _yMax << std::endl;
    _nextSamplingPoint[0] = _xMax;
    _nextSamplingPoint[1] = _yMin - _stepSize;

}



ObjectModelGrid::ObjectModelGrid(const std::string objectName):_objectName(objectName){
    _xMin = _xMax = _yMin = _yMax = _stepSize = 0;
    _nextSamplingPoint.resize(3);
    _nextSamplingPoint.zero();

    _stepSize = 50.0/1000.0; //TODO put it in a config file
    _searchSpaceWidth = 70.0/1000.0; //TODO put it in a config file
}
}
