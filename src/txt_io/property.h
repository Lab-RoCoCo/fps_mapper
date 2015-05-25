#pragma once

#include <Eigen/Core>
#include <string>
#include <tr1/memory>
#include <map>
#include <stdexcept>
#include <list>

namespace txt_io {

  class BaseProperty{
  public:
    BaseProperty(const std::string& name);
    virtual ~BaseProperty();
    inline const std::string& name() const {return _name;}
    virtual void push() = 0;
    virtual void pop() = 0;
  protected:
    std::string _name;
  };

  template <typename T>
  class Property : public BaseProperty{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef T ValueType;

    Property(const std::string name, const T& v) : BaseProperty(name){
      _values.push_front(v);
    }

    inline const T& value() const { return _values.front(); }

    void setValue(const T& v) { _values.front() = v; }

    virtual void push() {_values.push_front(_values.front());}
    virtual void pop()  {_values.pop_front();}

  protected:
    std::list<T, Eigen::aligned_allocator<T> > _values;
  };

  struct PropertyMap: public std::map<std::string, std::tr1::shared_ptr<BaseProperty> > {
    inline const BaseProperty* findProperty(const std::string& name) const {
      const_iterator it = find(name);
      if (it==end())
	return 0;
      return it->second.get();
    }

    inline BaseProperty* findProperty(const std::string& name) {
      iterator it = find(name);
      if (it==end())
	return 0;
      return it->second.get();
    }

    inline void pushProperty(const std::string& name) {
      BaseProperty* p=findProperty(name);
      if (! p) {
	std::string error("attempt to push the value of a non existing property: ");
	error = error + name;
	throw std::runtime_error(error.c_str());
      }
      p->push();
    }

    inline void popProperty(const std::string& name) {
      BaseProperty* p=findProperty(name);
      if (! p) {
	std::string error("attempt to pop the value of a non existing property: ");
	error = error + name;
	throw std::runtime_error(error.c_str());
      }
      p->pop();
    }
    
    template <typename T>
    const T& getProperty(const std::string& name) const {
      const BaseProperty* p=findProperty(name);
      if (p==0){
	std::string error("attempt to access the value of a non existing property: ");
	error = error + name;
	throw std::runtime_error(error.c_str());
      }
      const Property<T>* pt = dynamic_cast<const Property<T>* >(p);
      if (! pt) {
	std::string error("attempt to retrieve the value of a non existing property: ");
	error = error + name;
	throw std::runtime_error(error.c_str());
      }
      return pt->value();
    }

    template <typename T>
    void setProperty(const std::string& name, const T& v, bool ignore_if_present=false) {
      BaseProperty* p=findProperty(name);
      if (p==0){
	insert(make_pair(name, std::tr1::shared_ptr<Property<T> >(new Property<T>(name, v))));
	return;
      }
      if (ignore_if_present)
	return;
      Property<T>* pt = dynamic_cast<Property<T>* >(p);
      if (! pt) {
	throw std::runtime_error("attempt to set a value of wring type to a property");
      }
      pt->setValue(v);
    }

  };

}
