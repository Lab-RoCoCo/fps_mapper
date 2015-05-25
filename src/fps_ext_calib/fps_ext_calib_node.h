#pragma once
// H E A D E R S
//
//

// project headers
#include "tracker/multi_tracker.h"
#include "globals/system_utils.h"
#include "txt_io/message_reader.h"
//#include "txt_io/pinhole_image_message.h"
//#include "txt_io/static_transform_tree.h"
//#include "core/depth_utils.h"
//#include "tracker/base_triggers.h"
#include "ros_wrappers/image_message_listener.h"
#include "tracker/call_tracker_trigger.h"

#include "fps_ext_calib/ext_calib_config.h"
#include "fps_ext_calib/ext_calib.h"
#include "fps_ext_calib/ext_calib_viewer.h"
#include "fps_ext_calib/ext_calib_trigger.h"			// custom trigger for the tracker
#include "fps_ext_calib/ext_calib_msg_trigger.h"		// custom trigger for sensor messages

// opencv headers
/*
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <Eigen/Core>
*/

// c++ headers
#include <vector>
#include <cstdio>

// ros headers
#include <ros/ros.h>
#include <tf/transform_listener.h>





// C L A S S E S
//
//


namespace fps_mapper {
	
	
	/** @class main class for the calibration task. manages all other classes that are need. ros::init must have been called
	 *         before construction of this class. after construction, calling spin() will let this class do its work.
	 */
	class ExtCalibNode {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;	// not sure if needed
			//ExtCalibNode( ExtCalibConfig &config ) : config_(config), nh_("~"), img_tr_(nh_), q_app_(0), ec_viewer_(0) {
			//	init();
			//}
			ExtCalibNode( ExtCalibConfig &config, int argc, char **argv ) : config_(config), nh_("~"), img_tr_(nh_), q_app_(0), ec_viewer_(0), ext_calib_(config.n_topics_) {
				init( argc, argv );
			}
			
			/// initializes the object. requires the config to be set.
			void init( int argc=0, char **argv = NULL ) {
				// todo: set from config (and add the corresponding fields to the config in the first place)
				std::string tracker_cfg = "Xtion320x240";
				std::string tracker_aligner_type = "projective";
				
				// not sure if we need this sorter or not
				sorter_.setTimeWindow( 0.0 );
				
				// code that has been removed (commented out), because QApplication::processEvents would segfault while the code was present
				//
				
				// requirements for the ImageMessageListener object creation
				// moved to config_:
				//std::string base_link_frame_id = "base_link";
				//std::string odom_frame_id = "/odom";
				
				//
				// end of removed code
				
				tf::TransformListener *listener = new tf::TransformListener( ros::Duration(60.0) );
				
				// create a multi-tracker for each topic
				for( int i=0; i<config_.n_topics_; i++ ) {
					// note: we need a MultiTracker, not a Tracker object for this task. the MultiTrackers will be tracking only one topic each nonetheless.
					//       a Tracker object would listen to all (depth-)image topics, while the MultiTracker only listens to the topics set via init()
					printf( "adding multi-tracker for topic '%s'\n", config_.topic_names_[i].c_str() );
					
					// create the (multi-)tracker, one for each topic
					MultiTracker *mt = MultiTracker::makeTracker( tracker_aligner_type, tracker_cfg );
					if( !mt ) {
						printf( "failed to create tracker for topic '%s'\n", config_.topic_names_[i].c_str() );
						exit( 0 );
					}
					
					// set options for the multi-tracker, according to the config object
					mt->setBadPointsRatio( config_.bad_points_ratio_ );
					mt->aligner().solver().setDamping( config_.damping_ );
					mt->setImageShrink( config_.shrink_ );
					mt->setMaxDistance( config_.max_distance_ );
					mt->setMinDistance( config_.min_distance_ );
					
					triggers_.push_back( new ECTrigger(mt, 0xFF, i, config_) );
					// debugging
					//
					if( i == 0 ) {
						//test_trigger_ = new ECTrigger( mt, 0xFF, config_.n_topics_+0, config_ );
					}
					//
					// end of debugging
					
					//set the topic
					std::vector<std::string> single_topic_list;
					single_topic_list.push_back( config_.topic_names_[i]);
					mt->init( single_topic_list );
					
					// add the tracker to our internal list
					mtrackers_.push_back( mt );
					
					//fps_mapper::MsgTrigger* caller = new fps_mapper::CallTrackerTrigger( &sorter_, i+1, mt );
					fps_mapper::MsgTrigger* caller = new fps_mapper::MsgTrigger( &sorter_, i, mt, ext_calib_ );
					msg_triggers_.push_back( caller );
					//fps_mapper::CallTrackerTrigger* caller = new fps_mapper::CallTrackerTrigger( &sorter_, 0, mt );
					
					// setting up the listener for reading via txt_io interface
					ImageMessageListener* camera_listener = new ImageMessageListener( &nh_, &img_tr_, &sorter_, listener, config_.odom_frame_id_, config_.base_link_frame_id_ );
					camera_listener->subscribe( config_.topic_names_[i] );
					printf( "subscribing to topic: '%s'\n", config_.topic_names_[i].c_str() );
					cam_listeners_.push_back( camera_listener );
				}
				
				// create a viewer if the gui is enabled in the config
				if( config_.gui_ ) {
					//int argc = 0;
					//char *argv[] = { 0 };
					q_app_ = new QApplication( argc, argv );
					printf( "spawning viewer\n" ); fflush( stdout );
					ec_viewer_ = new ExtCalibViewer( q_app_, mtrackers_ );
					//ec_viewer_ = new ExtCalibViewer();
					printf( "ExtCalibViewer: constructor returned\n" ); fflush( stdout );
				}
			}
			
			/// calls spinOnce in a loop. breaks when ros::ok returns false.
			void spin() {
				printf( "starting spinning\n" );
				fflush( stdout );
				
				while( ros::ok() ) {
					spinOnce();
				}
			}
			
			/// calls ros::spin; resets trackers that are 'broken'
			void spinOnce() {
				ros::spinOnce();
				
				/**/
				// check if any of the trackers needs to be reset
				for( unsigned int i=0; i<mtrackers_.size(); i++ ) {
					if( mtrackers_[i]->isTrackBroken() ) {
						printf( "resetting tracker #%02i\n", i );
						mtrackers_[i]->clearStatus();
					}
				}
				
				if( ec_viewer_ ) {
					q_app_->processEvents();
					ec_viewer_->spinOnce();
				}
			}
			
		protected:
			/// object to store the options/configuration
			ExtCalibConfig &config_;
			/// node handle that is used by this class
			ros::NodeHandle nh_;
			/// vector of multi-trackers - one multi-tracker for each topic
			std::vector<MultiTracker*> mtrackers_;
			/// vector of image listeners, that feed (via callbacks) the MultiTracker objects
			std::vector<ImageMessageListener*> cam_listeners_;
			/// trigger objects that get invoked when sensor messages are broadcasted (via a SensorMessageSorter object)
			std::vector<MsgTrigger*> msg_triggers_;
			/// list of triggers for the trackers (fire on events such as ALIGNMENT_DONE or PATH_BROKEN)
			std::vector<ECTrigger*> triggers_;
			/// image transport object
			image_transport::ImageTransport img_tr_;
			/// sensor message sorter - required by the ImageMessageListener object
			txt_io::SensorMessageSorter sorter_;
			/// QT object that is required by the viewer GUI. processes external events such as mouse clicks or keyboard events.
			QApplication *q_app_;
			/// qt-based viewer that visualizes the point clouds from the sensors/trackers
			ExtCalibViewer *ec_viewer_;
			/// trigger for debug messages. not used at the moment
			ECTrigger *test_trigger_;
			
			ExtCalib ext_calib_;
	};




} // end of namespace fps_mapper




























