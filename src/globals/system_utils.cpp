#include "system_utils.h"
#include <iostream>
#include <fstream>

namespace system_utils {
  using namespace std;

  void printBanner(const char** banner) {
    const char** b = banner;
    while(*b) {
      std::cerr << *b << std::endl;
      b++;
    }
  }
  

  //! returns the system time in seconds
  double getTime() {
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv2sec(tv);
  }


  SystemUsageCounter::SystemUsageCounter(){
    gettimeofday(&_last_update_time, NULL);
    getrusage(RUSAGE_SELF, &_last_usage);
  }

  void SystemUsageCounter::update(){
    struct rusage current_usage;
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    getrusage(RUSAGE_SELF, &current_usage);
    
    struct timeval aux;
    // compute the total time elapsed between two update calls
    timersub(&_last_update_time, &current_time, &aux);
    double time_lapse = aux.tv_sec*1000+aux.tv_usec*1e-3;
    
    // compute the total cpu time in user space
    timersub(&_last_usage.ru_utime, &current_usage.ru_utime, &aux);
    double cpu_user_time = aux.tv_sec*1000+aux.tv_usec*1e-3;
    _user_cpu = cpu_user_time/time_lapse;
   
    // compute the total cpu time in system space
    timersub(&_last_usage.ru_stime, &current_usage.ru_stime, &aux);
    double cpu_system_time = aux.tv_sec*1000+aux.tv_usec*1e-3;
    _last_update_time = current_time;
    _last_usage = current_usage;


    std::ifstream is("/proc/self/statm");
    if (is) {
      // int _resident;
      // int _share;
      // int _text;
      // int _lib;
      // int _data;
      // int _dt;
      is >> _total_memory; // >> _resident >> _share >> _text >> _lib >> _data >> _dt;
      // cerr << endl;
      // cerr << "monitoring memory: " 
      // 	   << _total_memory << " "
      // 	   << _resident << " "
      // 	   << _share << " "
      // 	   << _text << " " 
      // 	   << _lib << " "
      // 	   << _data << " "
      // 	   << _dt << endl << endl;
    }else
      _total_memory = 0;
    is.close();

  }

}
