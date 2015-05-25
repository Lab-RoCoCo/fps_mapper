#pragma once



// H E A D E R S
//
//

// project headers
#include "tracker/multi_tracker.h"
#include "fps_ext_calib/ext_calib_config.h"



// C L A S S E S
//
//


namespace fps_mapper {
	
	
	
	
	/** @class custom trigger that is derived from the abstract Trigger class in the Tracker class. implements the action method
	 */
	class ECTrigger : public Tracker::Trigger {
		public:
			ECTrigger( Tracker* tracker, int event, int priority, ExtCalibConfig &config ) : Tracker::Trigger(tracker, event, priority), config_(config) {}
		
			virtual void action( Tracker::TriggerEvent ev ) {
				
				if( config_.debug_ ) {
					const char *ev_name = "???";
					switch( ev ) {
						case Tracker::NEW_FRAME_CREATED:
							ev_name = "NEW_FRAME_CREATED";
							break;
							
						case Tracker::NEW_CAMERA_ADDED:
							ev_name = "NEW_CAMERA_ADDED";
							break;
							
						case Tracker::ALIGNMENT_DONE:
							ev_name = "ALIGNMENT_DONE";
							break;
							
						case Tracker::TRACK_GOOD:
							ev_name = "TRACK_GOOD";
							break;
							
						case Tracker::TRACK_BROKEN:
							ev_name = "TRACK_BROKEN";
							break;
							
						case Tracker::TRACKING_DONE:
							ev_name = "TRACKING_DONE";
							break;
							
						case Tracker::PROCESSING_DONE:
							ev_name = "PROCESSING_DONE";
							break;
							
						case Tracker::REFERENCE_FRAME_RESET:
							ev_name = "REFERENCE_FRAME_RESET";
							break;
					}
					
					printf( "ECTrigger: action triggered, event=%02X (%s)\n", (int)ev, ev_name );
				}
				
			}
		
		
		protected:
			ExtCalibConfig &config_;
	};
	
	
} // end of namespace 'fps_mapper'























