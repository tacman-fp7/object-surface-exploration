#pragma once

// This object is used to update features which will be shared between object objectExploraton
// and object classification threads
// I can also add suggested trajectory which can be used by maintain contact module to 
// change trajectory/location
namespace objectExploration
{
  class ObjectFeatures
  {
  public:
    bool updateFeatures(){/*Do nothing at the moment*/};
    
  private:
    // A container for the features
  };
  
} // End of namespace