#pragma once
#include <vector>
#include <yarp/sig/Vector.h>

namespace objectExploration {

using yarp::sig::Vector;

class ObjectModelGrid{

public:
    ObjectModelGrid(const std::string objectName);
    ~ObjectModelGrid(){}

    //void setBoundingBox(const double xMin, const double xMax, const double yMin, const double yMax, const double stepSize);
    void addContactPoint(const yarp::sig::Vector fingertipPosition);
    void saveContactPoints();
    bool getNextSamplingPos(yarp::sig::Vector &nextSamplingPoint);
    void init(yarp::sig::Vector startingPos, yarp::sig::Vector endingPos);

private:
   std::string _objectName;

   std::vector< yarp::sig::Vector > _contactPoints;

   yarp::sig::Vector _nextSamplingPoint;
   double _searchSpaceWidth;
   double _stepSize;
   double _xMin;
   double _xMax;
   double _yMin;
   double _yMax;
};

}
