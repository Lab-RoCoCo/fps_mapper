

#include "bbox.h"
#include "boss/deserializer.h"
#include "boss/serializer.h"
#include "gl_helpers/simple_viewer.h"
#include "gl_helpers/opengl_primitives.h"
#include "fps_map/local_map.h"
#include <QApplication>

#include <vector>
#include <iostream>
#include <sstream>



namespace fps_mapper {



/** @class class that stores the config for the application */
class BBConfig {
  public:
    BBConfig() : input_map_(0), input_bbox_(0) {};
    
    /// filename of the map
    const char *input_map_;
    /// filename of the bounding boxes
    const char *input_bbox_;
};

/// serializes the transformation matrix into a string with a format like this: "[1 2 3; 0 -2 0; 0 0 1]"
template <typename T> std::string matToString( const T &mat ) {
	std::ostringstream oss;
	
	int r = mat.rows();
	int c = mat.cols();
	
	oss << "[";
	for( int i=0; i<r; i++ ) {
		if( i )
			oss << "; ";
		for( int j=0; j<c; j++ ) {
			if( j )
				oss << " ";
			oss << mat.matrix()(j, i);
		}
	}
	oss << "]";
	
	return oss.str();
}



BOSS_REGISTER_CLASS( BoundingBox );

/** @class viewer that displays the map and the bounding boxes */
class BBViewer : public GLHelpers::SimpleViewer{
  public:
    /// adds a local map
    void addLocalMap( LocalMap *lmap ) {
      static int counter = 0;
      counter++;
      if( counter > 28 )
        return;
      local_maps_.push_back( lmap );
    }
    
    /// adds a bounding box
    void addBox( BoundingBox &box ) {
      boxes_.push_back( box );
    }
    
    /// draws the local maps and bounding boxes
    void draw() {
      for( int i=0; i<local_maps_.size(); i++ ) {
        // not sure why this is needed. without this line, all but the first local map are drawn in white (on white background). this is ignored if the map contains rgbd data.
        glColor3f(0.2, 0.4, 0.4);
        
        local_maps_[i]->draw( ATTRIBUTE_SHOW | ATTRIBUTE_SELECTED | ATTRIBUTE_HIDE_REF );
        //local_maps_[i]->draw( ATTRIBUTE_SHOW | ATTRIBUTE_SELECTED );
      }
      
      for( int i=0; i<boxes_.size(); i++ ) {
        boxes_[i].draw( id );
      }
    }
    
    void keyPressEvent(QKeyEvent *e) {
      switch( e->key() ) {
        case Qt::Key_V: {     // select (and show) all local maps
          update();
        }
        
        case Qt::Key_S: {     // save to file
          std::string out_filename("test");
          std::cout << "writting to file '" << out_filename << "'" << std::endl;
          boss::Serializer ser;
          ser.setFilePath( out_filename );
          ser.setBinaryPath( out_filename + ".d/<classname>.<nameAttribute>.<id>.<ext>" );
          for( std::vector<BoundingBox>::iterator it = boxes_.begin(); it != boxes_.end(); ++it ) {
            ser.writeObject( *it );
          }
          std::cout << "finished writing" << std::endl;
        }
        
        case Qt::Key_L: {
          std::string in_filename( "test" );
          std::cout << "reading from file '" << in_filename << "'" << std::endl;
        }
        
        default:
          return;
      }
    }
    
    void postSelection( const QPoint& ) {
      int id = selectedName();
      std::cerr << "id=" << id << std::endl;
    }
  
  protected:
    std::vector<LocalMap*> local_maps_;
    std::vector<BoundingBox> boxes_;
};


} // end of namespace 'fps_mapper'




Eigen::Affine3f createRotMatrix(double ax, double ay, double az) {
  Eigen::Affine3f rx = Eigen::Affine3f(Eigen::AngleAxisf(ax, Eigen::Vector3f(1, 0, 0)));
  Eigen::Affine3f ry = Eigen::Affine3f(Eigen::AngleAxisf(ay, Eigen::Vector3f(0, 1, 0)));
  Eigen::Affine3f rz = Eigen::Affine3f(Eigen::AngleAxisf(az, Eigen::Vector3f(0, 0, 1)));
  return rz * ry * rx;
}

Eigen::Affine3f createScaleMatrix( float sx, float sy, float sz ) {
  Eigen::Matrix3f m( Eigen::Matrix3f::Identity() );
  Eigen::Affine3f a( Eigen::Affine3f::Identity() );
  
  m << Eigen::Vector3f( sx, 0, 0 ), Eigen::Vector3f( 0, sy, 0 ), Eigen::Vector3f( 0, 0, sz );
  
  a.linear() = m;
  
  return a;
}

Eigen::Affine3f createTransMatrix( float tx, float ty, float tz ) {
  Eigen::Affine3f m = Eigen::Affine3f::Identity();
  m.translation() = Eigen::Vector3f( tx, ty, tz );
  
  return m;
}




using namespace fps_mapper;



void test( BBViewer &viewer ) {
  
  printf( "loading test scene\n" );
  
  float pi = 3.14159265359;
  
  BoundingBox center;
  center.color( 0,0,0 ).deltaTranslate(4, 0, 0);
  viewer.addBox( center );
  
  float d = 1.1;
  for( int i=0; i<4; i++ ) {
    viewer.addBox( center.clone().color( 1,0,0 ).deltaTranslate( i*d, 0, 0 ) );
    viewer.addBox( center.clone().color( 0,1,0 ).deltaTranslate( 0, i*d, 0 ) );
    viewer.addBox( center.clone().color( 0,0,1 ).deltaTranslate( 0, 0, i*d ) );
  }
  
  BoundingBox c2 = center.clone();
  c2.deltaRotate( 0, 0, pi/6 ).scale( 0.8, 0.8, 0.8 ).deltaRotate( pi/12, 0, 0 );
  viewer.addBox( c2 );

  for( int i=0; i<4; i++ ) {
    viewer.addBox( c2.clone().color( 1,0,0 ).deltaTranslate( i*d, 0, 0 ) );
    viewer.addBox( c2.clone().color( 0,1,0 ).deltaTranslate( 0, i*d, 0 ) );
    viewer.addBox( c2.clone().color( 0,0,1 ).deltaTranslate( 0, 0, i*d ) );
  }
  /*
  */
}


int main( int argc, char** argv ) {
  QApplication app( argc, argv );
  
  fps_mapper::BBConfig config;
  fps_mapper::BBViewer viewer;
  
  // read the comamnd line parameters
  for( int i=0; i<argc; i++ ) {
    if( !strcmp(argv[i], "map") ) {
      i++;
      config.input_map_ = argv[i];
    } else if( !strcmp(argv[i], "bbox") ) {
      i++;
      config.input_bbox_ = argv[i];
    } else {
      //config.input_map_ = argv[i];
    }
  }
  
  boss::Deserializer des;
  boss::Serializable* o;
  
  // load the map
  if( config.input_map_ ) {
    printf( "loading the map\n" );
    des.setFilePath( config.input_map_ );
    while( (o = des.readObject()) ) {
      fps_mapper::LocalMap* lmap = dynamic_cast<fps_mapper::LocalMap*>(o);
      if( lmap )
        viewer.addLocalMap( lmap );
      
      fps_mapper::BoundingBox* bbox = dynamic_cast<fps_mapper::BoundingBox*>(o);
      if( bbox ) {
        viewer.addBox( *bbox );
      }
    }
    printf( "finished loading the map\n" );
  }
  
  // load the bounding boxes from file
  if( config.input_bbox_ ) {
    printf( "loading the bounding boxes\n" );
    boss::Deserializer des;
    des.setFilePath( config.input_bbox_ );
    boss::Serializable* o;
    
    while ( (o = des.readObject()) ) {
      printf( "." );
      fps_mapper::BoundingBox* bbox = dynamic_cast<fps_mapper::BoundingBox*>(o);
      if( bbox ) {
        viewer.addBox( *bbox );
      }
    }
    printf( "finshed loading the bounding boxes\n" );
  }
  
  
  //test( viewer );
  
  // old test code
  //
  if( 0 ) {
    fps_mapper::BoundingBox test_box;
    
    fps_mapper::BoundingBox test_box2;
    fps_mapper::BoundingBox test_box3;
    fps_mapper::BoundingBox test_box4;
    fps_mapper::BoundingBox test_box5;
    Eigen::Affine3f t( Eigen::Translation3f(Eigen::Vector3f(2,0,0)) );
    Eigen::Affine3f r = createRotMatrix( 1.0, 1.0, 1.0 );
    test_box2.aff_ = t * test_box2.aff_;
    test_box.color_[0] = 0.7;
    test_box.color_[1] = 0.7;
    test_box.color_[2] = 0.7;
    test_box2.color_[0] = 1.0;
    test_box2.color_[1] = 0.2;
    test_box2.color_[2] = 0.2;
    test_box3.color_[0] = 0.2;
    test_box3.color_[1] = 0.8;
    test_box3.color_[2] = 0.2;
    test_box4.color_[0] = 0.2;
    test_box4.color_[1] = 0.2;
    test_box4.color_[2] = 1.0;
    test_box5.color_[0] = 1.0;
    test_box5.color_[1] = 0.2;
    test_box5.color_[2] = 1.0;
    
    test_box3.aff_ = (Eigen::Translation3f(Eigen::Vector3f(0,2,0))) * test_box3.aff_;
    test_box4.aff_ = (Eigen::Translation3f(Eigen::Vector3f(0,0,2))) * test_box4.aff_;
    test_box5.aff_ = (Eigen::Translation3f(Eigen::Vector3f(2,2,2))) * createRotMatrix( 0.0, 0.3, 0.0 ) * test_box5.aff_;
    
    /*
    viewer.addBox( test_box );
    viewer.addBox( test_box2 );
    viewer.addBox( test_box3 );
    viewer.addBox( test_box4 );
    viewer.addBox( test_box5 );

    std::cerr << "bounding box #1 transformation:" << std::endl << fps_mapper::matToString(test_box.aff_.matrix()) << std::endl;
    std::cerr << "bounding box #2 transformation:" << std::endl << fps_mapper::matToString(test_box2.aff_.matrix()) << std::endl;
    std::cerr << "bounding box #3 transformation:" << std::endl << fps_mapper::matToString(test_box3.aff_.matrix()) << std::endl;
    std::cerr << "bounding box #4 transformation:" << std::endl << fps_mapper::matToString(test_box4.aff_.matrix()) << std::endl;
    std::cerr << "bounding box #5 transformation:" << std::endl << fps_mapper::matToString(test_box5.aff_.matrix()) << std::endl;
    */
  }
  

  if( 0 ) {  // create bounding boxes around the map center (0,0,0) that show the orientation
    viewer.addBox( BoundingBox().color( 0, 0, 0 ).scale( 0.5, 0.5, 0.5 ) );
    viewer.addBox( BoundingBox().color( 1, 0, 0 ).scale( 0.5, 0.5, 0.5 ).deltaTranslate( 1, 0, 0 ) );
    viewer.addBox( BoundingBox().color( 0, 1, 0 ).scale( 0.5, 0.5, 0.5 ).deltaTranslate( 0, 1, 0 ) );
    viewer.addBox( BoundingBox().color( 0, 0, 1 ).scale( 0.5, 0.5, 0.5 ).deltaTranslate( 0, 0, 1 ) );
  }
  
  float pi = 3.14159265359;
  
  /*
  // bounding box for the semantic map
  fps_mapper::BoundingBox bb_1;
  
  bb_1.color_[0] = 0.3;
  bb_1.color_[1] = 1.0;
  bb_1.color_[2] = 1.0;
  
  bb_1.aff_ = createTransMatrix( -7.6, 4.45, -0.7 ) * createRotMatrix(pi*0.02, pi*-0.03, pi*0.0) * createScaleMatrix( 0.6, 0.6, 0.7 );
  viewer.addBox( bb_1 );
  
  
  fps_mapper::BoundingBox bb_2;
  
  bb_2.color_[0] = 1.0;
  bb_2.color_[1] = 0;
  bb_2.color_[2] = 0;
  
  bb_2.aff_ = createTransMatrix( -7.6, 4.45, -0.9 ) * createRotMatrix(pi*0.02, pi*-0.03, pi*0.0) * createScaleMatrix( 0.6, 0.6, 0.7 );
  viewer.addBox( bb_2 );
  
  
  fps_mapper::BoundingBox bb_3;
  
  bb_3.color_[0] = 1.0;
  bb_3.color_[1] = 0.7;
  bb_3.color_[2] = 0;
  
  bb_3.aff_ = createTransMatrix( -7.6, 4.45, -1.1 ) * createRotMatrix(pi*0.02, pi*-0.03, pi*0.0) * createScaleMatrix( 0.6, 0.6, 0.75 );
  viewer.addBox( bb_3 );
  */
  
  
  if( 0 ) {
    
    fps_mapper::BoundingBox bb_chair_1;
    
    bb_chair_1.color_[0] = 1.0;
    bb_chair_1.color_[1] = 0.2;
    bb_chair_1.color_[2] = 1.0;
    
    bb_chair_1.aff_ = createTransMatrix( -7.6, 4.45, -1.1 ) * createRotMatrix(pi*0.02, pi*-0.03, pi*0.0) * createScaleMatrix( 0.6, 0.6, 0.75 );
    viewer.addBox( bb_chair_1 );
    
    /*
    fps_mapper::BoundingBox bb_chair_2;
    
    bb_chair_2.color_[0] = 1.0;
    bb_chair_2.color_[1] = 0.2;
    bb_chair_2.color_[2] = 1.0;
    
    bb_chair_2.iso_ = createTransIso( -7.6, 4.45, -1.1 ) * createRotIso(pi*0.02, pi*-0.03, pi*0.0);
    bb_chair_2.scale( 0.6, 0.6, 0.75 );
    bb_chair_2.scale( 0.3, 0.3, 0.3 );
    bb_chair_2.deltaTranslate( 2.5, 0, 0 );
    
    viewer.addBox( bb_chair_2 );
    
    fps_mapper::BoundingBox bb_chair_3;
    
    bb_chair_3.color_[0] = 1.0;
    bb_chair_3.color_[1] = 0.2;
    bb_chair_3.color_[2] = 1.0;
    
    bb_chair_3.iso_ = createTransIso( -7.6, 4.45, -1.1 ) * createRotIso(pi*0.02, pi*-0.03, pi*0.0);
    bb_chair_3.scale( 0.6, 0.6, 0.75 );
    bb_chair_3.scale( 0.3, 0.3, 0.3 );
    bb_chair_3.deltaTranslate( 2.0, 0, 0 );
    
    viewer.addBox( bb_chair_3 );
    */
  }
  
  if( 0 ) {
    
    fps_mapper::BoundingBox bb_bot_1;
    
    bb_bot_1.color_[0] = 0.5;
    bb_bot_1.color_[1] = 0.9;
    bb_bot_1.color_[2] = 0.2;
    
    bb_bot_1.aff_ = createTransMatrix( 1.3, -0.9, -0.4 ) * createRotMatrix(pi*-0.02, pi*-0.02, pi*0.4) * createScaleMatrix( 0.60, 0.60, 1.45 );
    viewer.addBox( bb_bot_1 );

    
    fps_mapper::BoundingBox bb_bot_2;
    
    bb_bot_2.color_[0] = 0.5;
    bb_bot_2.color_[1] = 0.9;
    bb_bot_2.color_[2] = 0.2;
    
    bb_bot_2.aff_ = createTransMatrix( 1.4, 0.65, -0.4 ) * createRotMatrix(pi*-0.02, pi*-0.02, pi*0.6) * createScaleMatrix( 0.60, 0.60, 1.45 );
    viewer.addBox( bb_bot_2 );
    
    
    fps_mapper::BoundingBox deck_chair;
    
    deck_chair.color_[0] = 1.0;
    deck_chair.color_[1] = 0.6;
    deck_chair.color_[2] = 0.1;
    
    deck_chair.aff_ = createTransMatrix( 0.8, 1.4, -0.3 ) * createRotMatrix(pi*-0.02, pi*-0.02, pi*0.28) * createScaleMatrix( 1.4, 0.70, 0.8 );
    viewer.addBox( deck_chair );
    
  }
  
  if( 0 ) {
  
    fps_mapper::BoundingBox bb_chair_1;
    
    bb_chair_1.color_[0] = 1.0;
    bb_chair_1.color_[1] = 0;
    bb_chair_1.color_[2] = 0;
    
    bb_chair_1.aff_ = createTransMatrix( 2.8, -1.66, 0 ) * createRotMatrix(pi*-0.06, pi*-0.06, pi*0.3) * createScaleMatrix( 0.5, 0.5, 0.5 );
    viewer.addBox( bb_chair_1 );
    
    fps_mapper::BoundingBox bb_chair_2;
    
    bb_chair_2.color_[0] = 1.0;
    bb_chair_2.color_[1] = 0;
    bb_chair_2.color_[2] = 0;
    
    bb_chair_2.aff_ = createTransMatrix( 3.7, -1.0, 0 ) * createRotMatrix(pi*-0.06, pi*-0.06, pi*0.3) * createScaleMatrix( 0.6, 0.6, 0.6 );
    viewer.addBox( bb_chair_2 );

    fps_mapper::BoundingBox bb_chair_3;
    
    bb_chair_3.color_[0] = 1.0;
    bb_chair_3.color_[1] = 0;
    bb_chair_3.color_[2] = 0;
    
    bb_chair_3.aff_ = createTransMatrix( 3.1, -0.6, 0 ) * createRotMatrix(pi*-0.06, pi*-0.06, pi*0.3) * createScaleMatrix( 0.6, 0.6, 0.6 );
    viewer.addBox( bb_chair_3 );

    fps_mapper::BoundingBox bb_chair_4;
    
    bb_chair_4.color_[0] = 1.0;
    bb_chair_4.color_[1] = 0;
    bb_chair_4.color_[2] = 0;
    
    bb_chair_4.aff_ = createTransMatrix( 2.0, -0.75, 0 ) * createRotMatrix(pi*-0.06, pi*-0.06, pi*0.3) * createScaleMatrix( 0.65, 0.65, 0.65 );
    viewer.addBox( bb_chair_4 );

    
    
    fps_mapper::BoundingBox bb_table_1;
    
    bb_table_1.color_[0] = 1.0;
    bb_table_1.color_[1] = 0.7;
    bb_table_1.color_[2] = 0;
    
    bb_table_1.aff_ = createTransMatrix( 3.1, -1.7, 0 ) * createRotMatrix(pi*-0.06, pi*-0.06, pi*0.3) * createScaleMatrix( 0.8, 1.5, 0.6 );
    viewer.addBox( bb_table_1 );
  }
  /*
  */
  //
  // end of test code
  
  viewer.show();
  app.exec();
}





















