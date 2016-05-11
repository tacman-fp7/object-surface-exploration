#include "objectModelGrid.h"
#include <assert.h>

namespace objectExploration {

void ObjectModelGrid::addContactPoint(const yarp::sig::Vector fingertipPosition){

   assert(fingertipPosition.size() == 3);
    std::cout << "Contact position: " << fingertipPosition.toString() << std::endl;
    _contactPoints.push_back(fingertipPosition);
}


void ObjectModelGrid::saveContactPoints(){

}

bool ObjectModelGrid::getNextSamplingPos(yarp::sig::Vector){

    return true;
}

void ObjectModelGrid::setBoundingBox(const double xMin, const double xMax,
                                     const double yMin, const double yMax){

    _xMin = xMin;
    _xMax = xMax;
    _yMin = yMin;
    _yMax = yMax;

}

}
