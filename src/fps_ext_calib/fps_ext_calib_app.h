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

// C++ headers
#include <unistd.h>




// C L A S S E S
//
//


namespace fps_mapper {
	
	
	enum {
		STATE_RUN,
		STATE_PAUSE,
		STATE_SEEK
	};
	
	
	class ExtCalibApp {
		public:
			ExtCalibApp( ExtCalibConfig &config ) : config_(config), q_app_(0), ec_viewer_(0), timestamp_start_(-1.0), state_(0), ext_calib_(config.n_topics_) {
				init();
			}
			
			void init( int argc=0, char **argv = NULL ) {
				// todo: set from config (and add the corresponding fields to the config in the first place)
				std::string tracker_cfg = "Xtion320x240";
				std::string tracker_aligner_type = "projective";
				
				// not sure if we need this or not
				sorter_.setTimeWindow( 0.0 );
				
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
					
					// trigger for debugging
					//triggers_.push_back( new ECTrigger(mt, 0xFF, i, config_) );
					
					//set the topic
					std::vector<std::string> single_topic_list;
					single_topic_list.push_back( config_.topic_names_[i]);
					mt->init( single_topic_list );
					
					// add the tracker to our internal list
					mtrackers_.push_back( mt );
					
					//fps_mapper::MsgTrigger* caller = new fps_mapper::CallTrackerTrigger( &sorter_, i+1, mt );
					//fps_mapper::MsgTrigger* caller = new fps_mapper::MsgTrigger( &sorter_, i, mt, ext_calib_ );
					//msg_triggers_.push_back( caller );
				}
				
				// create a viewer if the gui is enabled in the config
				if( config_.gui_ ) {
					q_app_ = new QApplication( argc, argv );
					printf( "spawning viewer\n" ); fflush( stdout );
					ec_viewer_ = new ExtCalibViewer( q_app_, mtrackers_ );
					printf( "ExtCalibViewer: constructor returned\n" ); fflush( stdout );
				}
				
				reader_.open( config_.in_filename_.c_str() );
				
				// for testing
				//
				state_ = STATE_SEEK;
				seek_pos_ = 300.0;
				//
				// end of testing
			}
				
			void spinOnce() {
				double time_now = 0.0;		// time passed since the start (in seconds)
				
				// process qt events
				if( ec_viewer_ ) {
					q_app_->processEvents();
					ec_viewer_->spinOnce();
				}
				
				// check if the read is ok (for example returns false if end of file has been reached)
				if( !reader_.good() ) {
					usleep( 10000 );
					return;
				}
				
				if( state_ == STATE_PAUSE )
					return;
				
				// read the next message
				txt_io::BaseMessage *msg;
				msg = reader_.readMessage();
				if( !msg )
					return;
				
				txt_io::BaseSensorMessage* sensor_msg = dynamic_cast<txt_io::BaseSensorMessage*>(msg);
				if( sensor_msg ) {
					double timestamp = sensor_msg->timestamp();
					if( timestamp_start_ == -1.0 )
						timestamp_start_ = timestamp;
					
					time_now = timestamp - timestamp_start_;
				} else {
					return;
				}
				
				if( state_ == STATE_SEEK ) {
					if( time_now < seek_pos_ )
						return;
					std::cout << "position seek finished" << std::endl;
					state_ = STATE_RUN;
				}
				std::cout << "\rt=" << time_now << "      ";
				
				txt_io::PinholeImageMessage* img = dynamic_cast<txt_io::PinholeImageMessage*>(msg);
				
				Matrix6f odom_info;
				//odom_info.setIdentity();
				odom_info.setZero();		// set the odometry weight to zero, so that the scan matcher will determine the movement alone
				
				if( !img->hasOdom() ) {
					odom_info.setZero();
				} else {
					std::cout << "odom=" << t2v(img->odometry()).transpose() << "   ";
				}
				
				Eigen::Isometry3f odom( img->odometry() );
				
				// reset odometry that is attached to the image
				img->setOffset(Eigen::Isometry3f::Identity());
				img->setOdometry(Eigen::Isometry3f::Identity());
				
				for( unsigned int i=0; i<config_.topic_names_.size(); i++ ) {
					if( img->topic() != config_.topic_names_[i] )
						continue;
					
					// process the frame
					mtrackers_[i]->processFrame( img->image(),
						RGBImage(),
						img->cameraMatrix(),
						img->depthScale(),
						img->seq(),
						img->timestamp(),
						img->topic(),
						img->frameId(),
						img->offset(),
						img->odometry(),
						odom_info
					);
					
					img->untaint();
					
					ext_calib_.push( i, img->timestamp(), mtrackers_[i]->globalT(), odom );
					
					if( mtrackers_[i]->isTrackBroken() ) {
						printf( "resetting tracker #%02i\n", i );
						mtrackers_[i]->clearStatus();
					}
					return;
				}
				
				//std::cout << "no match found for topic '" << img->topic() << "'" << std::endl;
				//exit( 0 );
				
				/*
				
				// marks the image as "not tainted", so no changes are written to the disk
				*/
			}
			
			void spin() {
				while( true )
					spinOnce();
			}
		
		protected:
			/// config object, containing the configuration/parameters for this class
			ExtCalibConfig config_;
			/// vector of multi-trackers - one multi-tracker for each topic
			std::vector<MultiTracker*> mtrackers_;
			/// trigger objects that get invoked when sensor messages are broadcasted (via a SensorMessageSorter object)
			std::vector<MsgTrigger*> msg_triggers_;
			/// list of triggers for the trackers (fire on events such as ALIGNMENT_DONE or PATH_BROKEN)
			std::vector<ECTrigger*> triggers_;
			/// reader object to read from files
			txt_io::MessageReader reader_;
			/// sensor message sorter - required by the ImageMessageListener object
			txt_io::SensorMessageSorter sorter_;
			/// QT object that is required by the viewer GUI. processes external events such as mouse clicks or keyboard events.
			QApplication *q_app_;
			/// qt-based viewer that visualizes the point clouds from the sensors/trackers
			ExtCalibViewer *ec_viewer_;
			/// timestamp of the first sensor message (message that contains a timestamp)
			double timestamp_start_;
			
			int state_;
			double seek_pos_;
			
			ExtCalib ext_calib_;
	};
	
	
	
} // end of namespace 'fps_mapper'



















