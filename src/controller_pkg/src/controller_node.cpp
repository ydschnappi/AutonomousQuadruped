#include <ros/ros.h>

#include <ros/console.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <algorithm>

#define PI M_PI

//define state here

#define CLOCKWISE_TURN 8
#define COUNTERCLOCKWISE_TURN -8
#define UPSTAIR 2
#define SLOPE 3
#define STOP -1


#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf provides useful functions to convert tf 
// messages to eigen types and vice versa, have a look to the documentation:
// http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html
#include <eigen_conversions/eigen_msg.h>

class controllerNode{
  ros::NodeHandle nh;


  ros::Subscriber desired_state, current_state;
  ros::Publisher prop_speeds;
  ros::Timer timer;

  // Set the required vectors to 2d

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector2d x;     // current position of the dog's c.o.m. in the world frame
  Eigen::Vector2d v;     // current velocity of the dog's c.o.m. in the world frame
  Eigen::Vector2d v_normalized;  // normalized v 

  // Desired state
  Eigen::Vector2d xd;    // desired position of the dog's c.o.m. in the world frame
  Eigen::Vector2d vd;    // desired velocity of the dog's c.o.m. in the world frame

  
  Eigen::Vector2d distance; // The distance vector from current position to desired position
  Eigen::Vector2d distance_normalized;  // normalized distance
  
  double cos_theta; // The cosine value of the angle between distance and the current velocity
  double scaler_rotate; // The scaler to scale the rotation angle
  double scaler_speed; // The scaler to scale the speed (speed gain)
  double amplitude; // The amplitude of the robot foot
  double scaler_distance2rot; // The scaler describing the effect of distance on rotational speed
  double base_rotate;
  double w;
  int state;

  double hz;             // frequency of the main control loop

public:
  controllerNode():hz(1000.0),state(0){
    desired_state = nh.subscribe("desired_state", 1, &controllerNode::onDesiredState, this);
    current_state = nh.subscribe("current_state_est", 1, &controllerNode::onCurrentState, this);
    prop_speeds = nh.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 1);
    // in a time period the controlLoop will be called
    timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
  }

  void onDesiredState(const trajectory_msgs::MultiDOFJointTrajectoryPoint& des_state){
    // Position
    geometry_msgs::Vector3 t = des_state.transforms[0].translation;
    xd << t.x, t.y;
    state = std::floor(t.z); //note the 3rd term is state
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
      
    x << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y;
    v << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y;
    
    // compute the 2d distance
    distance = xd - x;

    // set the speed gain to control the magnitude of the robot speed
    scaler_speed = std::min(8*std::sqrt(distance.dot(distance)), 1.0);
    scaler_speed = std::max(scaler_speed, 0.2);

    // stop moving if the robot is close enough to the target point
    if(std::sqrt(distance.dot(distance)) < 1e-2){
      scaler_speed = 0.0;
    }

    // the scaler that describes the effect of distance on rotational speed
    scaler_distance2rot = std::min(4*distance.dot(distance),3.0) + 
      0.05 / (distance.dot(distance) + 0.01);


    // compute cos_theta with dot product of distance_normalized and v_normalized
    // define also scaler_rotate
    // rotate only if v and distance are non-zero vectors
    if (std::sqrt(v.dot(v)) && std::sqrt(distance.dot(distance))){
      // normalize distance and v
      distance_normalized = distance / std::sqrt(distance.dot(distance));
      v_normalized = v / std::sqrt(v.dot(v));
      cos_theta = std::abs(distance_normalized.dot(v_normalized));
      // use cross product to difine the rotation direction
      if(v_normalized({0})*distance_normalized({1}) - v_normalized({1})*distance_normalized({0})<=0){
        scaler_rotate = 4.0 + scaler_distance2rot;
      }
      else{
        scaler_rotate = - 4.0 - scaler_distance2rot;
      }
      // when theta is too large or the target point is behind the robot, we need a faster rotation.
      // turning fast if target point is behind the robot or theta is too large
      if(distance_normalized.dot(v_normalized) < 0.8){
        cos_theta = 0.3;
        scaler_speed = 0.1;
      }

    }
    // if v or distance is zero vector, stop rotating
    else{
      scaler_rotate = 0.0;
      cos_theta = 1.0;
    }
    // ROS_INFO_NAMED("onCurrentState", "x:%f y:%f s:%f a:%f", x({0}), x({1}), std::sqrt(distance.dot(distance)), amplitude);
  }


  void controlLoop(const ros::TimerEvent& t){

    mav_msgs::Actuators msg;
    msg.angular_velocities.resize(4);
    switch(state){
      case CLOCKWISE_TURN:  //Turn clockwise
        msg.angular_velocities[0] = 0; 
        msg.angular_velocities[1] = 6; 
        msg.angular_velocities[2] = 1.0; 
        break;

      case COUNTERCLOCKWISE_TURN: //Turn counterclockwise
        msg.angular_velocities[0] = 0; 
        msg.angular_velocities[1] = -6; 
        msg.angular_velocities[2] = 1.0; 
        break;

      case UPSTAIR:
        msg.angular_velocities[0] = -1.0; 
        msg.angular_velocities[1] = 0;
        msg.angular_velocities[2] = 1.4;
        break;

      case SLOPE:
        msg.angular_velocities[0] = 1.0; 
        msg.angular_velocities[1] = 0;
        msg.angular_velocities[2] = 1.2;
        break;

      case STOP:
        msg.angular_velocities[0] = 0; 
        msg.angular_velocities[1] = 0;
        msg.angular_velocities[2] = 1;
        break;
        
      default:
        msg.angular_velocities[0] = scaler_speed; // Forward Velocity
        // When the controller is working, the rotational speed won't be exactly zero. If stabilized, there is 
        // a slight vibration at the desired orientation. This is to avoid too slow rotation when the 
        // current orientation is close to the desired orientation.
        msg.angular_velocities[1] = scaler_rotate * ( 1.3 - cos_theta); // Turning angle (max Value 4 - does not relate to a unit)
        msg.angular_velocities[2] = 1.0; // Amplitude
        msg.angular_velocities[3] = 0; // not used
    } 
    prop_speeds.publish(msg);
  }
};


int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
