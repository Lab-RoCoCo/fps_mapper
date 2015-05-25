#pragma once
#include "txt_io/sensor_message_sorter.h"
#include "tracker/tracker.h"
#include "txt_io/pinhole_image_message.h"
#include <cstdio>




namespace fps_mapper {
	
	/** @class a trigger class that listens to sensor messages from the SensorMessageSorter class. the virtual
	 *         'action' method is called when a sensor message is encountered. we provide our custom action
	 *         method here to call the tracker with reset odometry.
	 */
	class MsgTrigger : public txt_io::SensorMessageSorter::Trigger {
		public:
			MsgTrigger( txt_io::SensorMessageSorter* sorter, int priority, Tracker* tracker, ExtCalib &ext_calib ) :
				txt_io::SensorMessageSorter::Trigger(sorter, priority), tracker_(tracker), priority_(priority), ext_calib_(ext_calib) {
			}
			
			/// function that is called when the trigger is fired
			virtual void action( std::tr1::shared_ptr<txt_io::BaseSensorMessage> msg ) {
				txt_io::PinholeImageMessage* img = dynamic_cast<txt_io::PinholeImageMessage*>( msg.get() );
				if( !img )
					return;
				
				Matrix6f odom_info;
				odom_info.setIdentity();
				
				if( !img->hasOdom() ) {
					odom_info.setZero();
				}
				
				// reset odometry that is attached to the image
				img->setOffset(Eigen::Isometry3f::Identity());
				img->setOdometry(Eigen::Isometry3f::Identity());
				
				// process the frame
				tracker_->processFrame( img->image(),
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
				
				// marks the image as "not tainted", so no changes are written to the disk
				img->untaint();
				
				/*
				*/
				// debug message for the first topic (which has priority 0)
				if( priority_ == 0 )
					std::cerr << "T: " << t2v(tracker_->globalT()).transpose() << std::endl;
				
				ext_calib_.push( priority_, img->timestamp(), tracker_->globalT() );
			}
			
			/// getter function, returning the tracker
			inline Tracker* tracker() {
				return tracker_;
			}
			
		protected:
			Tracker* tracker_;
			int priority_;
			ExtCalib &ext_calib_;
	};


}	// end of namespace 'fps_mapper'
























