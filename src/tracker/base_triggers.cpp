#include <iostream>
#include "base_triggers.h"
namespace fps_mapper {

  

  VerboseTrigger::VerboseTrigger(Tracker* tracker, 
				 int event, 
				 int priorory,
				 const std::string& m):
    Tracker::Trigger(tracker, event, priorory) { 
    cerr<< " name: VerboseTrigger "<< endl;
    _message = m;
    _last_message  = "";
    _os = &std::cout;
  }

  void VerboseTrigger::action(Tracker::TriggerEvent ) {
    std::string tempMsg = _message;
    fillTagsMap();
    replaceTags(tempMsg);
    _last_message = tempMsg;
    if (_os)
      *_os << "Tracker: " << tempMsg << endl;
  }

  void VerboseTrigger::fillTagsMap() {
    char buf[1024];
    Vector6f p = t2v(_tracker->globalT());
    sprintf(buf, "%03f %03f %03f %03f %03f %03f ",
	    p[0], p[1], p[2], p[3], p[4], p[5]);
    _tags_map["global_pose"] = buf;
 
    p = t2v(_tracker->aligner().T());
    sprintf(buf, "%03f %03f %03f %03f %03f %03f ",
	    p[0], p[1], p[2], p[3], p[4], p[5]);
    _tags_map["local_pose"] = buf;

    p = t2v(_tracker->lastCameraOffset());
    sprintf(buf, "%03f %03f %03f %03f %03f %03f ",
	    p[0], p[1], p[2], p[3], p[4], p[5]);
    _tags_map["last_camera_offset"] = buf;

    p = t2v(_tracker->lastInitialGuess());
    sprintf(buf, "%03f %03f %03f %03f %03f %03f ",
	    p[0], p[1], p[2], p[3], p[4], p[5]);
    _tags_map["last_initial_guess"] = buf;

    _tags_map["last_initial_guess"] = _tracker->lastTopic();
     
    sprintf(buf,"%lf", _tracker->currentTime() - _tracker->startTime());
    _tags_map["total_time"] = buf;

    sprintf(buf,"%lf", _tracker->makeCloudTime());
    _tags_map["make_cloud_time"] = buf;

    sprintf(buf,"%lf", _tracker->alignmentTime());
    _tags_map["alignment_time"] = buf;

    sprintf(buf,"%lf", _tracker->validateTime());
    _tags_map["validate_time"] = buf;

    sprintf(buf,"%lf", _tracker->mergeTime());
    _tags_map["merge_time"] = buf;

    sprintf(buf,"%lf", _tracker->tailTime());
    _tags_map["tail_time"] = buf;

    sprintf(buf,"%lf", 1./(_tracker->currentTime() - _tracker->startTime()));
    _tags_map["fps"] = buf;

    sprintf(buf,"%06d", _tracker->lastSeq());
    _tags_map["seq"] = buf;

    sprintf(buf,"%d", _tracker->frameCount());
    _tags_map["frame_count"] = buf;
    
  }

  void VerboseTrigger::replaceTags(string& str) {
    string::iterator beginTag=str.end();
    for (string::iterator it=str.begin();it!=str.end();it++) {
      if (*it=='<') {
	beginTag=it;
	continue;
      }
      if (*it=='>'&&beginTag<it) {
	string replacement=_tags_map[str.substr(beginTag+1-str.begin(),it-beginTag-1)];
	size_t newpos=beginTag-str.begin()+replacement.length()-1;
	str.replace(beginTag,it+1,replacement);
	it=str.begin()+newpos;
      }
    }
  }
  
  SetMergingTrigger::SetMergingTrigger(Tracker* tracker, 
				       int event, 
				       int priorory,
				       bool enable):
    Tracker::Trigger(tracker, event, priorory) {
    cerr<< " name: SetMergigTrigger "<< endl;
    _enable = enable;
  }

  void SetMergingTrigger::action(Tracker::TriggerEvent ) {
    _tracker->enableMerging(_enable);
  }

  ClearStatusTrigger::ClearStatusTrigger(Tracker* tracker, 
					 int event, 
					 int priorory):
    Tracker::Trigger(tracker, event, priorory) {
    cerr<< " name: ClearStatusTrigger "<< endl;

  }
  
  void ClearStatusTrigger::action(Tracker::TriggerEvent) {
    _tracker->clearStatus();
  }

  ProfilerTrigger::ProfilerTrigger(fps_mapper::Tracker* tracker, 
				   int event, int priority,
				   system_utils::SystemUsageCounter* counter):
    Tracker::Trigger(tracker, event, priority){
    _usage_counter = counter;
    _count = 0;
    _window = 10;
  }
  
  void ProfilerTrigger::action(fps_mapper::Tracker::TriggerEvent e) {
    _count++;
    if (_count > _window) {
      _count = 0;
      _usage_counter->update();
    }
  }

}
