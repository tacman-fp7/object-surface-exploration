#pragma once
#include <vector>
#include <yarp/sig/Vector.h>

namespace objectExploration {

using yarp::sig::Vector;

class ObjectModelGrid{

public:
    ObjectModelGrid(const std::string objectName):_objectName(objectName){}


    /*(Vector startingPos, Vector endingPos, double stepSize, double width){
        _startingPos = startingPos;
        _endingPos = endingPos;
        _stepSize = stepSize;
        _width = width;

    }*/
    ~ObjectModelGrid(){}

    void setBoundingBox(const double xMin, const double xMax, const double yMin, const double yMax);
    void addContactPoint(const yarp::sig::Vector fingertipPosition);
    void saveContactPoints();
    bool getNextSamplingPos(yarp::sig::Vector);

private:
   std::string _objectName;

   std::vector< yarp::sig::Vector > _contactPoints;


   yarp::sig::Vector _startingPos;
   yarp::sig::Vector _endingPos;

   double _stepSize;
   double _xMin;
   double _xMax;
   double _yMin;
   double _yMax;
};

}
