#pragma once
#include <sys/time.h>
#include <sys/resource.h>



/**
 * @file   system_utils.h
 * @Author Giorgio Grisetti
 * @date   December 2014
 * @brief  This file contains some common routines and classes to get the system time, to print a banner on a comand line
 * and to monitor the process resource usage
 */
namespace system_utils {

  //! prints a null terminated array of char*.
  //! each item of the array prints on a separate line
  //! the last component of the array should be 0
  //! @param banner: the input char**
  void printBanner(const char** banner);

  //! returns the system time in seconds
  double getTime();

  //! converts timeval to sec
  inline double tv2sec(struct timeval& tv) { return (double) tv.tv_sec + 1e-6 * (double) tv.tv_usec;}

  //! monitors the resources of a process between two updates
  //! declare one instance of this object in the process you want to monitor
  //! and call once in a while the update() function
  class SystemUsageCounter{
  public:
    //! ctor;
    SystemUsageCounter();
    //! total system cpu usage in seconds;
    inline double systemCPUUsage() const { return _system_cpu; }
    //! total user cpu usage, in seconds
    inline double userCPUUsage() const { return _user_cpu; }
    //! total user cpu usage, in seconds
    inline double totalCPUUsage() const { return _user_cpu + _system_cpu; }
    //! total memory used by the project 
    inline int totalMemory() const { return _total_memory; }
    //! call this once in a while to update the internal counters
    void update();

  protected:
    struct timeval _last_update_time;
    struct rusage _last_usage;
  
    double  _system_cpu;
    double  _user_cpu;
    int _total_memory;

  };
}

