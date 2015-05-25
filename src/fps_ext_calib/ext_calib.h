// c++ headers
#include <vector>
#include <cstdio>
#include <queue>
#include <iostream>
#include <iomanip>


#include <Eigen/Core>


// C L A S S E S
//
//


namespace fps_mapper {
	
	
	/// a 6d point (translation and rotation), combined with a timestamp
	typedef std::pair< double, Eigen::Isometry3f> sensor_pos;
	
	/** @class this class stores "synchronized" key frames. because the cameras are in general not synchronized, we choose the closest
	 *         frames (in terms of timestamp) from the cameras instead. as a possible future feature, we could also add interpolation
	 *         for the frames. however interpolation is not required at the moment, and hence not implemented.
	 */
	class KeyFrames {
		public:
			KeyFrames( int n_topics ) : n_topics_(n_topics) {
				// minimum conditions to add a point to the queue, if any of these are met, we add the point to the queue (i.e. start a push operation)
				verbose_ = 0;
				min_translation_ = 0.02;		// 2cm
				min_angle_ = 10 * 0.01745;		// 10 degree
				min_time_ = 2.0;				// 2 seconds
				
				min_translation_ = 0.4;
				min_angle_ = 20 * 0.01745;
				min_time_ = 1.0;
				
				max_size_ = 10000;				// at most 10000 key frames per topic
				
				push_pending_ = 0;
				
				pos_.resize( n_topics_ );
				//last_updates_.resize( n_topics_ );
				last_updates_.resize( n_topics_, sensor_pos(0, Eigen::Isometry3f()) );
				awaiting_push_.resize( n_topics_, 0 );
			}
			
			/// called to add a sensor_pos as a potential key frame. the function determines whether or not the provided sensor_pos will be added as a key frame
			bool push( int id, sensor_pos pos, Eigen::Isometry3f &odom ) {
				// check if this is our very first read for this topic
				if( last_updates_[id].first == 0 ) {
					last_updates_[id] = pos;
					return false;
				}
				
				// check if there's a pending push operation
				if( push_pending_ ) {
					// check if this topic is already awaiting to be pushed
					if( awaiting_push_[id] )
						return false;
					
					// check if this topic or the one in last_updates is closer to the push timestamp
					double dt_old = push_timestamp_ - last_updates_[id].first;
					double dt_new = pos.first - push_timestamp_;
					if( fabs(dt_old) > fabs(dt_new ) ) {
						last_updates_[id] = pos;
					}
					awaiting_push_[id] = 1;
					push_pending_++;
					
					// check if we have collected all topics and are ready to actually push the topics to the queue
					if( push_pending_ == n_topics_ ) {
						pushPendingTopics();
						return true;
					}
					
					return false;
				}
				
				last_updates_[id] = pos;
				
				// check if we already have performed at least one push operation
				if( !pos_[id].empty() ) {
					// compute the motion for this topics since the last key frame
					Eigen::Isometry3f delta = pos.second * pos_[id].back().second.inverse();
					Eigen::AngleAxisf aa(delta.linear());
					double dt = push_timestamp_ - pos_[id].back().first;
					
					// check if we should start a push operation, i.e. check if the position has changed enough since the last key frame
					if( !(delta.translation().norm() < min_translation_  &&  fabs(aa.angle()) < min_angle_  &&  dt < min_time_) ) {
						push_timestamp_ = pos.first;
						awaiting_push_[id] = 1;
						push_pending_ = 1;
						// store a copy of the odometry
						odom_.push_back( Eigen::Isometry3f(odom) );
					}
					
					// check if we have collected all topics and are ready to actually push the topics to the queue
					if( push_pending_ == n_topics_ ) {
						pushPendingTopics();
						return true;
					}
					
					// debug message
					//
					
					// print only for the first topic (which has id 0)
					if( !id  && verbose_ ) {
						std::streamsize ss = std::cout.precision();
						std::cout << std::setprecision( 3 );
						std::cout// << "KeyFrames.push( " << id
							//<< ", " << (pos.first - last_updates_[id].first)
							<< ", t=" << delta.translation().norm()
							<< ", r=" << fabs(aa.angle())
							<< ", dt=" << (pos.first - pos_[id].back().first)
							<< ")"
							<< " T: " << t2v(pos_[id].back().second).transpose()
							<< " P: " << t2v(pos.second).transpose()
							<< " d: " << t2v(delta).transpose()
							<< std::endl;
						std::cout << std::setprecision( ss );
					}
					//
					// end of debug message
					
				} else {
					push_timestamp_ = pos.first;
					awaiting_push_[id] = 1;
					push_pending_ = 1;
					// store a copy of the odometry
					odom_.push_back( Eigen::Isometry3f(odom) );
				}
				
				return false;
			}
			
			void pop_front_n( int n ) {
				
				// remove the first n entries from odom
				for( int i=0; i<n; ++i ) {
					if( odom_.empty() )
						break;
					odom_.pop_front();
				}
				
				// remove the first n entries from each pos_ entry
				for( unsigned int k=0; k<pos_.size(); k++ ) {
					for( int i=0; i<n; i++ ) {
						if( pos_.empty() )
							break;
						
						pos_[k].pop_front();
					}
				}
				
			}
			
	//	protected:
			
			/// called when all topics are ready to be pushed on the queue. the topics are pushed and the related member variables are reset.
			void pushPendingTopics() {
				// iterate over all topics
				for( int i=0; i<n_topics_; i++ ) {
					// add entry from last_updates_ to the queue
					pos_[i].push_back( last_updates_[i] );
					
					// clear the flag
					awaiting_push_[i] = 0;
					
					// for testing: cut down the queue if the size is greater than 10
					//
					/*
					if( pos_[i].size() > 10 ) {
						for( int k=0; k<6; k++ ) {
							pos_[i].pop_front();
						}
					}
					*/
					//
					// end of testing
				}
				
				// for testing again
				/*
				if( odom_.size() > 10 ) {
					for( int k=0; k<6; k++ ) {
						odom_.pop_front();
					}
				}
				*/
				
				push_pending_ = 0;
				
				printf( "\npushing (%lu)\n", pos_[0].size() );
			}
			
			
			/// number of topics that we have
			int n_topics_;
			/// odometry for the corresposing timestamp
			std::list< Eigen::Isometry3f > odom_;
			/// position for topic at time index according to the tracker. the timestamp is saved inside the odom_point as a double
			std::vector< std::list<sensor_pos> > pos_;
			/// the latest topic's position. stored here in case we want to add it to the queue
			std::vector< sensor_pos > last_updates_;
			/// if a topic's position is ready to be pushed (i.e. the position and timestamp will not change until after the push operation), we set the according entry here to non-zero.
			std::vector< int > awaiting_push_;
			/// timestamp of the push (timestamp of the topic that introduced the push operation)
			double push_timestamp_;
			/// if non-zero, we want to push the topics, and at least one entry in awaiting_push_ is non-zero. the value is the number of topics that are ready to be pushed
			int push_pending_;
			
			/// if the translation between frames is at least that much, we will start a push operation
			double min_translation_;
			/// if the angle differs that much, we will start a push operation
			double min_angle_;
			/// if at least that much time has passed, we will start a push operation
			double min_time_;
			
			unsigned int max_size_;
			
			/// verbosity level: 0=errors only, 1=debug
			int verbose_;
	};
	
	
	
	/** @class a class to collect the positions of the sensors, including the timestamps
	 */
	class ExtCalib {
		public:
			ExtCalib( int n_topics ) : n_topics_(n_topics), key_frames_(n_topics) {
				//pos_.resize( n_topics );
				
				/*
				// minimum conditions to add a point to the queue, if any of these are meet, we add the point to the queue
				min_translation_ = 0.02;		// 2cm
				min_angle_ = 10 * 0.01745;		// 10 degree
				min_time_ = 2.0;				// 2 seconds
				
				// minimum deltas to perform an update
				up_min_translation_ = 0.25;		// 25 cm
				up_min_angle_ = 20 * 0.01745;	// 20 degree
				*/
			}
			
			void push( int id, double timestamp, Eigen::Isometry3f pos, Eigen::Isometry3f odom = Eigen::Isometry3f::Identity() ) {
				//if( !id )
				//	std::cout << "ExtCalib.push(" << id << ", " << timestamp << ")" << std::endl;
				if( key_frames_.push( id, sensor_pos({timestamp, pos}), odom ) ) {
					computeExtrinsics();
				}
				return;
				
				/*
				Eigen::Isometry3f delta = pos.inverse() * pos_[id].back().second;
				Eigen::AngleAxisf aa(delta.linear());
				double dt = timestamp - pos_[id].back().first;
				
				// if we have moved less than 2cm and rotated less than 10 degree and less than 2 seconds have passed, then we ignore the point
				if( delta.translation().norm() < min_translation_  &&  fabs(aa.angle()) < min_angle_  &&  dt < min_time_ )
					return;
				
				pos_[id].push( {timestamp, pos} );
				
				// if this is the first topic, we check if we should compute an update. otherwise we just return right here
				if( id != 0 )
					return;
				
				// compute the deltas between the first point in the queue and the current position. note, that all values are positive
				double delta_time = timestamp - pos_[id].front().first;		// should be positive unless the timestamps are messed up
				Eigen::Isometry3f delta_vec = pos.inverse() * pos_[id].front().second;
				double delta_trans = delta_vec.translation().norm();
				double delta_angle = fabs( (Eigen::AngleAxisf(delta_vec.linear())).angle() );
				
				// check if the angle is large enough to compute the normals
				if( delta_angle >= up_min_angle_ ) {
					// todo (1): check if angle is consistant amoung all topics
					// todo (2): compute normals for all topics
					// todo (3): align normals of all topics, update confidence values
				}
				
				if( delta_trans >= up_min_translation_ ) {
					// todo (1): check delta_angle - if small, assume translation only
					// todo (2): ...
				}
				
				// check if we need to shrink the queue
				if( delta_time >= 10.0  &&  pos_[0].size() > 10000 ) {
					
					// reduce size by 10 percent by removing elements from the front. do so on all topics (using the first topic for computing the new size)
					unsigned int new_queue_size = (pos_[0].size() * 9) / 10;
					for( int i=0; i<n_topics_; i++ ) {
						while( pos_[0].size() > new_queue_size ) {
							pos_[0].pop();
						}
					}
				}
				*/
			}
			
			void computeExtrinsics() {
				if( key_frames_.odom_.size() < 2 )
					return;
				
				// print the current values for the transformation from first to last
				std::cout << "angle axis for:" << std::endl;
				// compute rotation for odometry topic
				Eigen::Isometry3f odom_delta = key_frames_.odom_.front().inverse() * key_frames_.odom_.back();
				Eigen::AngleAxisf odom_aa( odom_delta.linear() );
				std::cout << "  " << "odom"
					<< " " << odom_aa.axis().transpose()
					<< "  (" << odom_aa.angle()
					<< ")  trans: " << odom_delta.translation().transpose()
					<< std::endl;
				
				// compute rotation for all camera topics
				for( unsigned int i=0; i<key_frames_.pos_.size(); i++ ) {
					Eigen::Isometry3f start = key_frames_.pos_[i].front().second;
					Eigen::Isometry3f end = key_frames_.pos_[i].back().second;
					Eigen::Isometry3f delta = start.inverse() * end;				// transformation from 'start' to 'end'
					
					Eigen::AngleAxisf aa( delta.linear() );
					
					// print result to console
					std::cout << "  " << i								// index of topic (0 = first topic, 1 = second topic, ...)
						<< "     " << aa.axis().transpose()				// the axis vector for the rotation
						<< "  (" << aa.angle()							// the angle of the rotation
						<< ", " << ( aa.angle() / odom_aa.angle() )		// comparison against the rotation obtained through the odometry
						<< ")  trans: " << delta.translation().transpose()	// translation vector
						<< std::endl;
				}
				
				// print the angles for each step (odom)
				std::cout << "angles (" << key_frames_.odom_.size() << "):" << std::endl;
				for( auto it_odom = key_frames_.odom_.begin(); ; ) {
					Eigen::Isometry3f start = *it_odom;
					++it_odom;
					if( it_odom == key_frames_.odom_.end() )
						break;
					
					Eigen::Isometry3f delta = start.inverse() * *it_odom;
					Eigen::AngleAxisf aa( delta.linear() );
					
					std::cout << "  " << aa.angle();
				}
				std::cout << std::endl;
				
				// print the angles for each step (sensors)
				for( unsigned int i=0; i<key_frames_.pos_.size(); i++ ) {
					for( auto it = key_frames_.pos_[i].begin(); ; ) {
						Eigen::Isometry3f start = it->second;
						++it;
						if( it == key_frames_.pos_[i].end() )
							break;
						
						Eigen::Isometry3f delta = start.inverse() * it->second;
						Eigen::AngleAxisf aa( delta.linear() );
						
						std::cout << "  " << aa.angle();
					}
					std::cout << std::endl;
				}
				
				// remove keyframes from the front that don't have similar angles
				checkAndClean();
				
				std::cout << "debug info: " << key_frames_.odom_.size() << ", " << key_frames_.pos_[0].size() << std::endl;
				
				computeR();
				/*
				while( true ) {
					char c;
					std::cin >> c;
					switch( c ) {
						case 'q':
							key_frames_.pop_front_n(1);
							break;
						default:
							return;
					}
				}
				*/
			}
			
			/// checks if the camera topics are consistent with each other (wheather or not the rotation angles are similar)
			void checkAndClean() {
				std::vector<float> angles;
				
				// compute rotation for all camera topics
				for( unsigned int i=0; i<key_frames_.pos_.size(); i++ ) {
					Eigen::Isometry3f start = key_frames_.pos_[i].front().second;
					Eigen::Isometry3f end = key_frames_.pos_[i].back().second;
					Eigen::Isometry3f delta = start.inverse() * end;				// transformation from 'start' to 'end'
					
					Eigen::AngleAxisf aa( delta.linear() );
					
					angles.push_back( aa.angle() );
				}
				
				if( angles.size() < 2 )
					return;
				
				float min = 1.0, max = 1.0;
				
				for( unsigned int i=0; i+1<angles.size(); i++ ) {
					for( unsigned int k=1; k<angles.size(); k++ ) {
						float temp = angles[i] / angles[k];
						if( temp < min )
							min = temp;
						if( temp > max )
							max = temp;
					}
				}
				
				std::cout << "check result: min=" << min << ", max=" << max << std::endl;
				
				if( min > 0.85  &&  max < 1.0/0.85 )
					return;
				
				unsigned int size = key_frames_.odom_.size();
				key_frames_.pop_front_n( size-1 );
			}
			
			// computes the rotation matrix to align the camera topics with the odometry
			void computeR() {
				Eigen::Isometry3f odom = key_frames_.odom_.front().inverse() * key_frames_.odom_.back();
				Eigen::AngleAxisf odom_aa( odom.linear() );
				Eigen::Matrix3f r_o;
				Eigen::Vector3f OY( odom.translation() );
				OY.normalize();
				Eigen::Vector3f OX( odom_aa.axis().cross(OY) );
				r_o << OX, OY, odom_aa.axis();
				
				
				// iterate over all camera topics
				for( unsigned int i=0; i<key_frames_.pos_.size(); i++ ) {
					Eigen::Isometry3f iso = key_frames_.pos_[i].front().second.inverse() * key_frames_.pos_[i].back().second;
					Eigen::AngleAxisf aa( iso.linear() );
					
					Eigen::Vector3f Z( aa.axis() );
					Eigen::Vector3f Y( iso.translation() );
					Y.normalize();
					Eigen::Vector3f X( Z.cross(Y) );
					
					// matrix representing the coordinate system with the translation vector and the rotation axis as y and z axis
					Eigen::Matrix3f r_c;
					r_c << X, Y, Z;
					
					Eigen::Matrix3f R = r_o.transpose() * r_c;
					
					if( i==1 ) {
						std::cout << "R: " << R << std::endl;
					}
				}
			}
			
		protected:
			/// number of topics that we have
			int n_topics_;
			/// object that stores "synchronized" keyframes for the topics
			KeyFrames key_frames_;
			
			/*
			/// position for topic at time index according to the tracker. the timestamp is saved inside the odom_point as a double
			std::vector< std::queue<sensor_pos> > pos_;
			/// the latest topic's position. stored here in case we want to add it to the queue
			std::vector< sensor_pos > last_updates_;
			/// if we want to push a topic's position, we set the according entry here to non-zero
			std::vector< int > awaiting_push_;
			/// if non-zero, we want to push the topics, and at least one entry in awaiting_push_ is non-zero. the value is the number of topics that are ready to be pushed
			int push_pending_;
			
			double up_min_translation_;
			double up_min_angle_;
			
			double min_translation_;
			double min_angle_;
			double min_time_;
			*/
	};
	
	
	
} // end of namespace 'fps_mapper'



























