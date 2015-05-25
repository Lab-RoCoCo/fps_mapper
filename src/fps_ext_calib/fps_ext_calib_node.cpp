// H E A D E R S
//
//

// project headers
#include "fps_ext_calib_node.h"


using namespace std;
using namespace txt_io;
using namespace fps_mapper;
using namespace Eigen;
using namespace system_utils;




// G L O B A L   V A R I A B L E S
//
//

const char* banner[]= {
  "fps_ext_calib_node: tool for calibration of rgbd sensors extrinsics",
/*  "usage:",
  " fps_tracker_gui_app [options] <dump filename>",
  " where: ",
  "  -aligner:      [string] aligner type [projective or nn], default: projective",
  "  -config:       [string] config type [Xtion640x480, Kinect640x480, Xtion320x240], default: Xtion320x240",
  "  -max_distance: [float] max range of the beams to consider for alignment, default 3",
  "  -min_distance: [float] min range of the beams to consider for alignment, default 0",
  "  -cam_only:     flag, if set ignores the odometry and operates in the camera frame",
  "  -t:            [string] specifies which image topic to use, if unset will use all",
  "                          to issue multiple topics use \"-t <topic1>  -t <topic2> .. -t <topicN> \"",
  "  -single        [flag] if set uses the single projector, otherwise it uses the multi projector on all topics",
  "  -tf            [string] if set overrides the transforms in the file with theones of the file provided",
  "  -bpr:          [float] bad points ratio [float], default: 0.1",
  "  -damping:      [float] solver damping, default: 100",
  "  -shrink :      [int]   image downscaling (2 means half size), default: 1",
  "  -o:            [string] output filename where to write the model, default \"\"",
  "once the gui has started you can:",
  " -dump the current cloud (W key)",
  " -pause/start the tracker(P key)",
  " -reset the cloud        (R key)",*/
  0
};



// F U N C T I O N S
//
//



int main( int argc, char **argv ) {
	
	// options/config for this node
	ExtCalibConfig config;
	
	// set default values
	config.n_topics_ = 0;
	config.bad_points_ratio_ = 0.1;
	config.damping_ = 100;
	config.shrink_ = 1;
	config.min_distance_ = 0;
	config.max_distance_ = 5;
	config.debug_ = 0;
	config.base_link_frame_id_ = "base_link";
	config.odom_frame_id_ = "/odom";
	
	// read the command line options
	for( int i=1; i<argc; i++ ) {
		if( !strcmp(argv[i], "-h") ) {
			printBanner( banner );
			return 0;
		} else if( !strcmp(argv[i], "-shrink") ) {
			i++;
			config.shrink_ = atoi( argv[i] );
		} else if( !strcmp(argv[i], "-damping") ) {
			i++;
			config.damping_ = atof( argv[i] );
		} else if( !strcmp(argv[i], "-bpr") ) {
			i++;
			config.bad_points_ratio_ = atof( argv[i] );
		} else if( !strcmp(argv[i], "-t") ) {
			i++;
			config.addTopic( argv[i] );
		} else if( !strcmp(argv[i], "-gui") ) {
			config.gui_ = 1;
		} else if( !strcmp(argv[i], "-debug") ) {
			config.debug_ = 1;
		} else {
			printf( "WARNING: command line option \"%s\" not recognized - ignored\n", argv[i] );
		}
	}
	
	config.printConfig();
	
	ros::init(argc, argv, "fps_tracker_node");
	
	ExtCalibNode node( config, argc, argv );
	
	node.spin();
	
}





































