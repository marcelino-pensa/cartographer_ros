
#ifndef CARTOGRAPHER_LPF_CLASS_H_
#define CARTOGRAPHER_LPF_CLASS_H_

#include "Eigen/Core"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"

#include "cartographer_ros/msg_conversion.h"

namespace lpf {

class SecondOrderFilter3d{
 public:
    Eigen::Vector2d X_x, X_y, X_z;
    ros::Time time_;
    Eigen::Matrix2d I_2x2; // Identity matrix

    //Second order low pass filter continuous model matrices
	Eigen::Matrix2d A;
	Eigen::Vector2d B;

    // Constructors
    SecondOrderFilter3d() {}
    SecondOrderFilter3d(const geometry_msgs::TransformStamped &init_point,
    	                const double &time_constant) {
    	// Set position/velocity/acceleration initial states
    	X_x =  Eigen::Vector2d(init_point.transform.translation.x, 0.0);
    	X_y =  Eigen::Vector2d(init_point.transform.translation.y, 0.0);
    	X_z =  Eigen::Vector2d(init_point.transform.translation.z, 0.0);
    	time_ = init_point.header.stamp;
    	
    	// Low pass filter parameters
    	const double zeta = 1.0;	            //Critically damped
		const double wn = 1.0/time_constant;	//Time constant = 1/(zeta.wn)
		I_2x2 = Eigen::Matrix2d::Identity(2,2);
		A << 0,      1,
		     -wn*wn, -2*zeta*wn;
		B << 0,
		     wn*wn;
    }

    geometry_msgs::TransformStamped Filter(const geometry_msgs::TransformStamped measurement) {
    	const double dt = measurement.header.stamp.toSec() - time_.toSec();
    	geometry_msgs::TransformStamped cur_state;
    	cur_state = measurement;

    	if (dt < 0) {  // Return current state
    		cur_state.header.stamp = ros::Time(time_);
    		cur_state.transform.translation = 
    			cartographer_ros::ToGeometryMsgVector3(X_x[0], X_y[0], X_z[0]);
    		return cur_state;
    	}

    	//Propagate states (crude euler integration)
		geometry_msgs::Vector3 meas_state = measurement.transform.translation;
		X_x = (I_2x2 + A*dt)*X_x + dt*B*meas_state.x;
		X_y = (I_2x2 + A*dt)*X_y + dt*B*meas_state.y;
		X_z = (I_2x2 + A*dt)*X_z + dt*B*meas_state.z;

    	time_ = measurement.header.stamp;
    	cur_state.transform.translation = 
    			cartographer_ros::ToGeometryMsgVector3(X_x[0], X_y[0], X_z[0]);
    	return cur_state;
    }

};

}  // namespace lpf

#endif  // CARTOGRAPHER_LPF_CLASS_H_