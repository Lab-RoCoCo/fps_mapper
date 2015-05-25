#pragma once
#include "tracker.h"
#include "globals/system_utils.h"

namespace fps_mapper {
  class VerboseTrigger: public Tracker::Trigger{
  public:
    VerboseTrigger(Tracker* tracker, int event, int priorory, const std::string& message = std::string(""));
    virtual void action(Tracker::TriggerEvent e);
    inline void setMessage(const std::string& msg) {_message = msg;}
    inline const std::string& message() const {return _message; }
    inline std::ostream * outputStream() { return _os; }
    inline void setOutputStream(std::ostream* os) {_os = os;}
    const std::string& lastMessage() { return _last_message; }
  protected:
    void fillTagsMap();
    void replaceTags(string& str);
    std::ostream* _os;
    std::string _message;
    std::string _last_message;
    std::map<std::string, std::string> _tags_map;
  };


  class SetMergingTrigger: public Tracker::Trigger{
  public:
    SetMergingTrigger(Tracker* tracker, 
		      int event, 
		      int priorory,
		      bool enable);
    virtual void action(Tracker::TriggerEvent e);
  protected:
    bool _enable;
  };

  class ClearStatusTrigger: public Tracker::Trigger{
  public:
    ClearStatusTrigger(Tracker* tracker, 
		       int event, 
		       int priorory);
    virtual void action(Tracker::TriggerEvent e);
  };


  class ProfilerTrigger: public fps_mapper::Tracker::Trigger{
  public:
    ProfilerTrigger(fps_mapper::Tracker* tracker, 
		    int event, int priority,
		    system_utils::SystemUsageCounter* counter) ;

    virtual void action(fps_mapper::Tracker::TriggerEvent e);

    inline system_utils::SystemUsageCounter* usageCounter() {return _usage_counter;}
  protected:
    int _count;
    int _window;
    system_utils::SystemUsageCounter* _usage_counter;
    
  };

}
