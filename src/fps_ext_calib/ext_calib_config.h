#pragma once


// H E A D E R S
//
//

// c++ headers
#include <vector>
#include <cstdio>









namespace fps_mapper {


	/** @class class to store the options/configuration for thie ExtCalibNode class 
	 */
	class ExtCalibConfig {
		public:
			ExtCalibConfig() : n_topics_(0), bad_points_ratio_(0.1), damping_(100), shrink_(1), min_distance_(0.0), max_distance_(3.5), gui_(1), debug_(1) {};
			/// number of topics
			int n_topics_;
			/// vector of topic names
			std::vector<std::string> topic_names_;
			/// bad point ratio: 0.1 means that we consider the tracker broken if we have 10% "bad points". use 1.0 to never have yout track broken.
			double bad_points_ratio_;
			/// damping for the solver
			double damping_;
			/// shrink factor (integer). 2 means half the size (in both dimensions of the depth image). use 1 to not shrink the input.
			int shrink_;
			/// points that are closer than this are discarded
			double min_distance_;
			/// points that are further away than this are discarded
			double max_distance_;
			/// name of the base frame (e.g. 'base_link'). used by the ImageMessageListener object
			std::string base_link_frame_id_;
			/// name of the odom frame (e.g. 'odom'). used by the ImageMessageListener object
			std::string odom_frame_id_;
			/// if non-zero we spawn a qt-based GUI
			int gui_;
			/// filename for data input (not used in the ROS node)
			std::string in_filename_;
			/// if non-zero, we print additional debug messages
			int debug_;
			
			/// adds a topic name to the list of stored (camera) topic names
			void addTopic( const char *topic ) {
				if( debug_ )
					printf( "adding topic '%s' to config\n", topic );
				
				topic_names_.push_back( std::string(topic) );
				n_topics_++;
			}
			
			/// prints the content of this class to the console
			void printConfig() {
				printf( "ext_calib config:\n" );
				printf( "  %s: %i\n", "n_topics", n_topics_ );
				for( unsigned int i=0; i<topic_names_.size(); i++ ) {
					printf( "  topic %02i: %s\n", i, topic_names_[i].c_str() );
				}
				printf( "  %s: %f\n", "bad_points_ratio", bad_points_ratio_ );
				printf( "  %s: %f\n", "damping", damping_ );
				printf( "  %s: %i\n", "shrink", shrink_ );
				printf( "  %s: %f\n", "min_distance", min_distance_ );
				printf( "  %s: %f\n", "max_distance", max_distance_ );
				printf( "  %s: %s\n", "in_filename", in_filename_.c_str() );
				printf( "  %s: %i\n", "gui", gui_ );
				printf( "  %s: %i\n", "debug", debug_ );
			}
	};
	
	
} // end of namespace 'fps_mapper'
















