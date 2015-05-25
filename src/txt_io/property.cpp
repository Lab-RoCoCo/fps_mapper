#include "property.h"

namespace txt_io{

  BaseProperty::BaseProperty(const std::string& n) {
    _name = n;
  }
  BaseProperty::~BaseProperty() {}

  void test_prop(){
    PropertyMap pmap;
    pmap.setProperty("intp", 10);
    pmap.setProperty("floatp", 10.5f);
    pmap.setProperty("vecp", Eigen::Vector3f(0.1, 0.1, 0.1));
    
    int i = pmap.getProperty<int>("intp");
    float f  = pmap.getProperty<float>("floatp");
    Eigen::Vector3f v = pmap.getProperty<Eigen::Vector3f>("vecp");
  }

}
