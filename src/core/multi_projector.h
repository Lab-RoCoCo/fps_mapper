#pragma once
#include "cloud.h"
#include "base_projector.h"
#include <stack>

namespace fps_mapper {
  using namespace std;

  /**
     This class encapsulates algorithm and parameters to project a 3D model onto an image,
     through a pinhole model.
   */
  class MultiProjector : public BaseProjector{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MultiProjector();

    //! ctor, initializes the inner fields
    //! the projectors are  OWNED by the multi guy
    MultiProjector(std::vector<BaseProjector*>& projectors_);


    virtual ~MultiProjector();

    /** does the projection, taking into account the normals and the occlusions.
	@param zbuffer: a float matrix used to handle occlusions, filled by the method
	@param indices: an int image, filled by the algorithm. Each cell contains the index of the point in model, that projects onto a pixel
	@param T: the transform world_to_camera applied before the projection
	@param model: input
     */
    virtual void project(FloatImage& zbuffer, 
		 IndexImage& indices,
		 const Eigen::Isometry3f& T,
		 const Cloud& model) const;

    // Throws and exception
    virtual void project(FloatImage& zbuffer, 
			 IndexImage& indices,
			 const Eigen::Isometry3f& T,
			 const FloatImage& src_zbuffer, 
			 const IntImage& src_indices,
			 float src_scale=1,
			 int subsample = 1) const;

    
    virtual void unproject(Cloud& cloud, const RawDepthImage& depth_image, const RGBImage& rgb_image=RGBImage());
    

    //! overridden from base class
    //! applies a scaling factor to the camera, not the image  
    //! computes the center so that it is the center of the image
    //! @param s: the scale (s=0.5 corresponds to half the image)
    virtual void scaleCamera(float s);


    //! overridden from base class Pushes the state of all projectors in the pool
    virtual void pushState();

    //! overridden from base class pops the state of all projectors in the pool
    virtual void popState();
    
    virtual void setImageSize(int r, int c);

    inline std::vector<BaseProjector*>& projectors() { return _projectors; }

    virtual void setCameraInfo(BaseCameraInfo* _camera_info);

    virtual void initFromCameraInfo(BaseCameraInfo* _camera_info);

    virtual void setMinDistance(float d);

    virtual void setMaxDistance(float d);

  protected:
    // performs the inverse depth projection, to be specialized in derived classes
    virtual void unprojectPoints(const RawDepthImage& depth_image);

    std::vector<BaseProjector*> _projectors;
    int _mono_rows;

    struct State {
      State(const Eigen::Isometry3f& offset, int rows, int cols);
      int image_rows;
      int image_cols;
      Eigen::Isometry3f offset;
    };

    std::stack<State, std::deque< State, Eigen::aligned_allocator<State> > > _states;

  };

}
