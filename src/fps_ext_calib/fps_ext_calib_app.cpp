// H E A D E R S
//
//

// project headers
#include "fps_ext_calib_app.h"


// G L O B A L   V A R I A B L E S
//
//

const char* banner[]= {
  "fps_ext_calib_app: tool for calibration of rgbd sensors extrinsics",
  "usage:",
  " fps_ext_calib_app [options]",
  " where: ",
  "  -aligner:      [string] aligner type [projective or nn], default: projective",
  "  -config:       [string] config type [Xtion640x480, Kinect640x480, Xtion320x240], default: Xtion320x240",
  "  -max_distance: [float] max range of the beams to consider for alignment, default 3",
  "  -min_distance: [float] min range of the beams to consider for alignment, default 0",
  "  -t:            [string] specifies which image topic to use",
  "                          to issue multiple topics use \"-t <topic1>  -t <topic2> .. -t <topicN> \"",
  "  -bpr:          [float] bad points ratio [float], default: 0.1",
  "  -damping:      [float] solver damping, default: 100",
  "  -shrink :      [int]   image downscaling (2 means half size), default: 1",
  "  -o:            [string] output filename where to write the model, default \"\"",
  "once the gui has started you can:",
  " -dump the current cloud (W key)",
  " -pause/start the tracker(P key)",
  " -reset the cloud        (R key)",/**/
  0
};


int main(int argc, char ** argv) {
	
	fps_mapper::ExtCalibConfig config;
	
	//txt_io::MessageReader reader;
	for( int i=0; i<argc; i++ ) {
		if( !strcmp(argv[i], "-h") ) {
			system_utils::printBanner( banner );
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
		} else if( !strcmp(argv[i], "in_filename") ) {
			i++;
			config.in_filename_ = std::string(argv[i]);
		} else if( !strcmp(argv[i], "-debug") ) {
			config.debug_ = 1;
		}
	}
	
	config.gui_ = 0;
	config.printConfig();
	
	fps_mapper::ExtCalibApp calib( config );
	std::cout << "starting to spin" << std::endl;
	calib.spin();
	
	return 0;
}






















