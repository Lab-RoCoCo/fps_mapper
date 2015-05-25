// H E A D E R S
//
//
#include "fps_ext_calib/ext_calib_viewer.h"
#include "gl_helpers/opengl_primitives.h"




//  M E T H O D S
//
//
namespace fps_mapper {
	
	
	using namespace GLHelpers;
	
	
	// draw the pointclouds from all the trackers into a single window
	void ExtCalibViewer::draw() {
		
		// iterate over all trackers
		for( unsigned int i=0; i<mtrackers_.size(); i++ ) {
			
			// check if the tracker has valid data - if not, skip it
			if( !mtrackers_[i]->currentModel()  ||  !mtrackers_[i]->referenceModel() )
				continue;
			
			glPushMatrix();
			{
				glMultMatrix( mtrackers_[i]->globalT() );
				// draw the reference system
				glPushMatrix();
				glScalef( 0.2, 0.2, 0.2 );
				GLHelpers::drawReferenceSystem();
				glPopMatrix();
				
				// draw pyramid wireframe
				glPushMatrix();
				glMultMatrix( mtrackers_[i]->lastCamera()->offset() );
				float pyrH = 0.1;
				float pyrW = 0.05;
				drawPyramidWireframe( pyrH, pyrW );
				glPopMatrix();
				
				// draw the reference cloud (gray)
				if( mtrackers_[i]->referenceModel() ) {
					glColor3f( 0.5, 0.5, 0.5 );
					mtrackers_[i]->referenceModel()->draw();
				}
				
				// draw the current cloud after applying the epsilon T from aligner
				glPushMatrix();
				glMultMatrix(mtrackers_[i]->aligner().T().inverse() );
				if( mtrackers_[i]->currentModel() ) {
					switch( i ) {
						default:	// fall through
						case 0:
							glColor3f( 0.2, 0.2, 1.0 );			// blue
							break;
						case 1:
							glColor3f( 0.1, 0.9, 0.1 );			// green
							break;
						case 2:
							glColor3f( 1.0, 0.2, 0.2 );			// red
							break;
					}
					mtrackers_[i]->currentModel()->draw();
				}
				glPopMatrix();
			}
			glPopMatrix();
			
		}
	}
	
	
	
}




















