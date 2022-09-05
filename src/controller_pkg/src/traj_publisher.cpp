//Author: Dian Yuan
//Comments:
//16.07.2022 Simple position and velocity publisher
//29.07.2022 trajectory visualization

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

class traj_publisher{
  ros::NodeHandle nh;
  ros::Publisher traj_pub, traj_vis_pub, current_goal_pub; 
  ros::Subscriber waypoint_sub, pos_sub;
  ros::Time start;
  ros::Timer timer; //timer for main signal loop

  double hz; //frequency of the signal publishing

  geometry_msgs::Point waypoint;
  visualization_msgs::Marker traj, current_goal; //Marker in rviz

public:
  traj_publisher():hz(50.0){
  
    waypoint_sub = nh.subscribe("next_waypoint",1,&traj_publisher::get_waypoint,this);
    pos_sub = nh.subscribe("current_state_est", 100, &traj_publisher::getTraj, this);
    traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state",1);
    traj_vis_pub = nh.advertise<visualization_msgs::Marker>("traj_marker",100);
    current_goal_pub = nh.advertise<visualization_msgs::Marker>("current_goal",1);
    start = ros::Time::now();
    timer = nh.createTimer(ros::Duration(1/hz), &traj_publisher::traj_pub_Loop, this);

    traj.header.frame_id = current_goal.header.frame_id = "world";
    traj.header.stamp = ros::Time::now();
    traj.ns = current_goal.ns = "traj_publisher";
    traj.id = 33;
    traj.action = visualization_msgs::Marker::ADD;
    traj.pose.orientation.w = 1.0;
    traj.type = visualization_msgs::Marker::LINE_STRIP;
    traj.scale.x = traj.scale.y = traj.scale.z = 0.04;
    traj.color.r = 0.99;
    traj.color.g = 0.65;
    traj.color.a = 0.88;
    traj.lifetime = ros::Duration();

    current_goal.id = 66;
    current_goal.action = visualization_msgs::Marker::ADD;
    current_goal.scale.x = current_goal.scale.y = current_goal.scale.z = 0.08;
    current_goal.color.r = current_goal.color.a = 1.0;
    current_goal.type = visualization_msgs::Marker::CUBE;
    current_goal.lifetime = ros::Duration(1/hz);

  }

  void get_waypoint(const geometry_msgs::Point & next_waypoint){
    waypoint = next_waypoint;
  }

  void getTraj(const nav_msgs::Odometry& cur_state){
    geometry_msgs::Point p;
    p.x = cur_state.pose.pose.position.x;
    p.y = cur_state.pose.pose.position.y;
    p.z = 0;
    traj.points.push_back(p);
  }


 
  void traj_pub_Loop(const ros::TimerEvent& t){
    trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
    msg.transforms.resize(1);
    msg.transforms[0].translation.x = waypoint.x;
    msg.transforms[0].translation.y = waypoint.y;
    msg.transforms[0].translation.z = waypoint.z;

    current_goal.header.stamp = ros::Time();
    current_goal.pose.position.x = waypoint.x;
    current_goal.pose.position.y = waypoint.y;
    current_goal.pose.position.z = 0;
    current_goal.pose.orientation.w = 1.0;

    traj_pub.publish(msg);
    traj_vis_pub.publish(traj);
    current_goal_pub.publish(current_goal);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "traj_publisher");
  traj_publisher n;
  ros::spin();
  return 0;
}
