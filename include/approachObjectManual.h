#pragma once

#include <approachObject.h>

namespace objectExploration
{



class ApproachObjectManual: public objectExploration::ApproachObject
{
public:
  ApproachObjectManual();
  virtual bool estimateInitContactPos();
  virtual bool approach();
};

}