#pragma once
// H E A D E R S
//
//

// qt headers
#include "qglviewer.h"
#include "qapplication.h"
#include <QDateTime>

// project headers
#include "gl_helpers/simple_viewer.h"
#include "tracker/multi_tracker.h"












namespace fps_mapper {


	
	class ExtCalibViewer : public GLHelpers::SimpleViewer {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;	// not sure if needed
			ExtCalibViewer( QApplication *q_app,  std::vector<MultiTracker*> &mtrackers, int max_fps=30 ) :  q_app_(q_app), mtrackers_(mtrackers), max_fps_(max_fps) {
			//ExtCalibViewer() {
				printf( "calling 'show'\n" ); fflush( stdout );
				show();
				usleep( 1000000 );
				printf( "done calling 'show'\n" ); fflush( stdout );
				
			}
			
			/// draws the clouds from the trackers
			virtual void draw();
			
			/** called to handle user input, redraws, etc. 
			 * 
			 * @note the redraw frequency is limited, by default to about 30 fps
			 */
			void spinOnce() {
				/*
				// limit the redraw frequency to about 30 fps max
				QDateTime now = QDateTime::currentDateTime();
				qint64 td = now.toMSecsSinceEpoch() - last_redraw_.toMSecsSinceEpoch();
				if( td < 1000/max_fps_ ) {
					return;
				}
				
				last_redraw_ = now;
				last_update_ = QDateTime::currentDateTime();
				*/
				updateGL();		// calls our virtual draw() function
				//q_app_->processEvents();	// might work now - used to cause a seg-fault for unknown reasons
			}
			
		protected:
			QApplication *q_app_;
			std::vector<MultiTracker*> &mtrackers_;
			QDateTime last_redraw_;
			int max_fps_;
	};
	
	
} // end of namespace 'fps_mapper'



















