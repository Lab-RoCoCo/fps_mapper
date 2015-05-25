#pragma once
#include "boss/eigen_boss_plugin.h"
#include "boss/serializable.h"
#include "gl_helpers/simple_viewer.h"
#include "gl_helpers/opengl_primitives.h"
#include <string>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>




namespace fps_mapper {
  
  
  /// creates a scale matrix from three floats
  Eigen::Affine3f createScaleAff( float sx, float sy, float sz ) {
    Eigen::Matrix3f m( Eigen::Matrix3f::Identity() );
    Eigen::Affine3f a( Eigen::Affine3f::Identity() );
    
    m << Eigen::Vector3f( sx, 0, 0 ), Eigen::Vector3f( 0, sy, 0 ), Eigen::Vector3f( 0, 0, sz );
    
    a.linear() = m;
    
    return a;
  }
  
  /// creates an rotation matrix from three floats
  Eigen::Isometry3f createRotIso(double ax, double ay, double az) {
    Eigen::Isometry3f rx = Eigen::Isometry3f(Eigen::AngleAxisf(ax, Eigen::Vector3f(1, 0, 0)));
    Eigen::Isometry3f ry = Eigen::Isometry3f(Eigen::AngleAxisf(ay, Eigen::Vector3f(0, 1, 0)));
    Eigen::Isometry3f rz = Eigen::Isometry3f(Eigen::AngleAxisf(az, Eigen::Vector3f(0, 0, 1)));
    return rz * ry * rx;
  }
  
  /// creates a translation matrix from three floats
  Eigen::Isometry3f createTransIso( float tx, float ty, float tz ) {
    Eigen::Isometry3f m = Eigen::Isometry3f::Identity();
    m.translation() = Eigen::Vector3f( tx, ty, tz );
    
    return m;
  }


  /** @class representation of a single bounding box */
  class BoundingBox : public boss::Serializable {
    public:
      /// constructor, creating a 1x1x1 bounding box qith one corner at (0,0,0) and the other corner at (1,1,1)
      BoundingBox() : iso_(Eigen::Isometry3f::Identity()), aff_(Eigen::Affine3f::Identity()), show_helper_(false) {
        //std::cerr << "bounding box transformation:" << std::endl << matToString(iso_.matrix()) << std::endl;
        
        scaling_ << 1,1,1;
        
        // default color: light blue
        color_[0] = 0.35;
        color_[1] = 0.35;
        color_[2] = 0.95;
      }
      
      /// copy constructor. creates a deep copy of the object
      BoundingBox( const BoundingBox &other ) : scaling_(other.scaling_), iso_(other.iso_), aff_(other.aff_), show_helper_(other.show_helper_) {
        color_[0] = other.color_[0];
        color_[1] = other.color_[1];
        color_[2] = other.color_[2];
      }
      
      /// draws the bounding box. todo: test the helpers
      void draw() {
        glColor3f(color_[0], color_[1], color_[2]);
        
        glPushAttrib(GL_LINE_WIDTH);
        glLineWidth(3);
        
        glPushMatrix();
        GLHelpers::glMultMatrix( aff_ );

        glBegin(GL_LINES);
        for( int i=0; i<2; i++ ) {
          for( int k=0; k<2; k++ ) {
            for( int t=0; t<2; t++ ) {
              glNormal3f(0,0,1);
              glVertex3f( 0+i, 0+k, 0 );
              glVertex3f( 0+i, 0+k, 0+t );
              
              glNormal3f(0,0,1);
              glVertex3f( 0+i, 0, 0+t );
              glVertex3f( 0+i, 0+k, 0+t );
              
              glNormal3f(0,0,1);
              glVertex3f( 0, 0+k, 0+t );
              glVertex3f( 0+i, 0+k, 0+t );
            }
          }
        }
        glEnd();
        
        if( show_helper_ ) {
          glBegin(GL_LINES);
          
          glColor3f( 1.0, 0.0, 0.0 );
          glVertex3f( 0.5, 0.5, 0.5 );
          glVertex3f( 1.5, 0.5, 0.5 );
          
          glColor3f( 0.0, 1.0, 0.0 );
          glVertex3f( 0.5, 0.5, 0.5 );
          glVertex3f( 0.5, 1.5, 0.5 );
          
          glColor3f( 0.0, 0.0, 1.0 );
          glVertex3f( 0.5, 0.5, 0.5 );
          glVertex3f( 0.5, 0.5, 1.5 );
          
          glEnd();
        }
        
        glPopMatrix();
        
        glPopAttrib();  // GL_LINE_WIDTH
        //glPopAttrib();  // GL_COLOR
        
      }
      
      void draw( int name ) {
        if( name != -1 ) {
          glPushName(name);
          draw();
          glPopName();
          return;
        }
        
        draw();
      }
      
      /// changes the size of the bounding box
      BoundingBox& scale( float sx, float sy, float sz ) {
        Eigen::Vector3f t = iso_.translation();
        
        // save new scaling
        scaling_ << sx, sy, sz;
        
        // compute new affine transformation matrix
        aff_ = iso_ * createScaleAff( sx, sy, sz );
        
        return *this;
      }
      
      /// moves the bounding box relative to its current position, taking the current orientation into account
      BoundingBox& deltaTranslate( float dx, float dy, float dz ) {
        Eigen::Vector3f dt;
        dt << dx, dy, dz;
        
        dt = iso_.linear() * dt;
        
        iso_.translation() += dt;
        
        aff_ = iso_ * createScaleAff( scaling_[0], scaling_[1], scaling_[2] );
        
        return *this;
      }
      
      /// rotates the bounding box
      BoundingBox& deltaRotate( float ax, float ay, float az ) {
        
        Eigen::Isometry3f rot = createRotIso( ax, ay, az );
        
        iso_ = iso_ * rot;
        aff_ = iso_ * createScaleAff( scaling_[0], scaling_[1], scaling_[2] );
        
        return *this;
      }
      
      /// sets the color of the bounding box
      BoundingBox& color( float r, float g, float b ) {
        color_[0] = r;
        color_[1] = g;
        color_[2] = b;
        
        return *this;
      }
      
      /// returns a copy of the bounding box object
      BoundingBox clone() {
        BoundingBox bb( *this );
        return bb;
      }
      
      /// serializes the object for storing in a text file
      void serialize( boss::ObjectData& data, boss::IdContext& context ) {
        //boss::ObjectData * bbData=new boss::ObjectData();
        data.setString( std::string("label"), label_ );
        
        iso_.matrix().toBOSS( data, "iso" );
        scaling_.toBOSS( data, "size" );
        
        boss::ArrayData* color_array = new boss::ArrayData;
        for( int i=0; i<3; i++ ) {
          color_array->push_back( new boss::NumberData(color_[i]) );
        }
        data.setField( std::string("color"), color_array );
      }
      
      /// deserializes the object from a text string
      void deserialize( boss::ObjectData& data, boss::IdContext& context ) {
        iso_.matrix().fromBOSS( data, "iso" );
        scaling_.fromBOSS( data, "size" );
        
        boss::ArrayData& color_array = data.getField( "color" )->getArray();
        for( unsigned int i=0; i<3 && i<color_array.size(); i++ ) {
          boss::ValueData& v = color_array[i];
          color_[i] = v.getFloat();
        }
        
        // compute new affine transformation matrix
        aff_ = iso_ * createScaleAff( scaling_[0], scaling_[1], scaling_[2] );
      }
      
      /// @brief required by the boss framework, but not needed in this class
      void deserializeComplete() {
      }
      
      /// the human-readable label
      std::string label_;
      /// size of the bounding box
      Eigen::Vector3f scaling_;
      /// transformation from the unit cube to the actual bounding box, defined as a 3x3 matrix
      Eigen::Isometry3f iso_;
      /// matrix, combining the scale and the transformation
      Eigen::Affine3f aff_;
      /// color of the bounding box
      float color_[3];
      /// if true, the coordination system is displayed
      bool show_helper_;
    protected:
  };

  
  
  
}















































