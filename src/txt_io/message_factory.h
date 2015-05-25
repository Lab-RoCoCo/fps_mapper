#pragma once
#include <string>
#include <iostream>
#include "base_message.h"
#include <map>
#include <stdexcept>
#include <iostream>
namespace txt_io {


  class MessageFactory{
  public:
    struct BaseMessageCreator{
      virtual BaseMessage* create() = 0;
      virtual ~BaseMessageCreator();
    };

    template <typename T> 
    struct MessageCreator: public MessageFactory::BaseMessageCreator{
      virtual BaseMessage* create() {
	return new T;
      }
    };
    
    static MessageFactory* instance();

    BaseMessage* create(const std::string& tag);

    template <typename T>
    void registerMessage() {
      T* msg=new T;
      std::string tag = msg->tag();
      delete msg;
      std::map<std::string, BaseMessageCreator*>::iterator it  = _creators.find(tag);
      if (it != _creators.end()) {
	throw std::runtime_error("error, type already registered");
      }
      std::cerr << "txt_io: Registering  message with tag [" << tag <<  "]" << std::endl;
      MessageCreator<T> *creator = new MessageCreator<T>;
      _creators.insert(std::make_pair(tag, creator));
    }

    template <typename T>
    struct MessageRegisterer{
      MessageRegisterer() {
	MessageFactory::instance()->registerMessage<T>();
      }
    };

  protected:
    static MessageFactory* _instance;
    std::map<std::string, BaseMessageCreator*> _creators;
  };

  
}
