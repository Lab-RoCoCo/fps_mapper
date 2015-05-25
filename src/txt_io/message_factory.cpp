#include <string>
#include <iostream>
#include <map>
#include "message_factory.h"

namespace txt_io {

  
  MessageFactory::BaseMessageCreator::~BaseMessageCreator(){};

  MessageFactory* MessageFactory::_instance = 0;

    
  MessageFactory* MessageFactory::instance() { 
    if (! _instance)
      _instance = new MessageFactory;
    return _instance;
  }

  BaseMessage* MessageFactory::create(const std::string& tag){
    std::map<std::string, BaseMessageCreator*>::iterator it  = _creators.find(tag);
    if (it == _creators.end())
      return 0;
    return it->second->create();
  }


}
