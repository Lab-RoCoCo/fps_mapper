
#pragma once
#include "base_sensor_message.h"
#include <limits>
#include <deque>
#include <queue>
#include <vector>
#include <tr1/memory>
#include <map>

namespace txt_io {
  class SensorMessageSorter{
  public:
    SensorMessageSorter(double time_window=1.0f);
    ~SensorMessageSorter();

    void insertMessage(BaseSensorMessage* msg);

    inline double timeWindow() const {return _time_window;}
  
    inline void setTimeWindow(double time_window) {_time_window = time_window;}

    inline bool writeBackEnabled() const  { return _write_back_enabled; }
    inline void setWriteBackEnabled(bool wbe) {_write_back_enabled = wbe;}
  protected:

  public:
    class Trigger{
    public:
      Trigger(SensorMessageSorter* sorter, int priority);
      virtual ~Trigger();
      virtual void action(std::tr1::shared_ptr<BaseSensorMessage> msg) = 0;
      inline SensorMessageSorter* sequencer() {return _sorter;}
      inline int priority() const {return _priority;}
    protected:
      SensorMessageSorter* _sorter;
      int _priority;
    };
    
    friend class Trigger;

  protected:

    typedef std::multimap<double, std::tr1::shared_ptr<BaseSensorMessage> >  DoubleMessageMap;
    double _time_window;
    double _oldest_time;
    double _newest_time;
    DoubleMessageMap _messages;
    bool _write_back_enabled;
    void callTriggers(std::tr1::shared_ptr<BaseSensorMessage> msg);
    std::map<int, Trigger*> _triggers;
  };

}
