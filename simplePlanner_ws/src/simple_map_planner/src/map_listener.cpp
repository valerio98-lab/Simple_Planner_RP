
#include <ros/ros.h> 
#include <nav_msgs/OccupancyGrid.h> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "planning_alg.h"

class MapListener {
  public:
    MapListener(ros::NodeHandle& n):planner(), n_(n){
      sub_map = n_.subscribe("map", 1000, &MapListener::mapCallback, this);
      ROS_INFO("map_listener: Subscribed to map topic");
      sub_init = n_.subscribe("initialpose", 1000, &MapListener::initialPoseCallback, this);
      ROS_INFO("map_listener: Subscribed to initialpose topic");
      sub_goal = n_.subscribe("move_base_simple/goal", 1000, &MapListener::goalPoseCallback, this);
      ROS_INFO("map_listener: Subscribed to move_base_simple/goal topic");

    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
      ROS_INFO("map_listener: mapCallback");
      map_received = true;
      map = *msg;
      Path_planning();
      ROS_INFO("Received map: %d X %d", msg->info.width, msg->info.height);
    }

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
      ROS_INFO("map_listener: initialPoseCallback");
      initial_pose = *msg;
      initial_pose_received = true;
      Path_planning();
      ROS_INFO("Received initial pose: %0.2f, %0.2f, %0.2f, %0.2f", 
                                                initial_pose.pose.pose.position.x, 
                                                initial_pose.pose.pose.position.y, 
                                                initial_pose.pose.pose.orientation.x, 
                                                initial_pose.pose.pose.orientation.y); 
    }

    void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
      ROS_INFO("map_listener: goalPoseCallback");
      goal_pose = *msg;
      goal_pose_received = true;
      Path_planning();
      ROS_INFO("Received goal pose: %0.2f, %0.2f, %0.2f, %0.2f", 
                                            goal_pose.pose.position.x, 
                                            goal_pose.pose.position.y, 
                                            goal_pose.pose.orientation.x, 
                                            goal_pose.pose.orientation.y);

    }

    void Path_planning() {
      if(map_received && initial_pose_received && goal_pose_received){
        planner.planPath(initial_pose, goal_pose, map);
      }
    }
  private:
    ros::NodeHandle n_;
    PlanningAlg planner;
    ros::Subscriber sub_map;
    ros::Subscriber sub_init;
    ros::Subscriber sub_goal;
    geometry_msgs::PoseStamped goal_pose;
    geometry_msgs::PoseWithCovarianceStamped initial_pose;
    nav_msgs::OccupancyGrid map;
    bool map_received = false;
    bool initial_pose_received = false;
    bool goal_pose_received = false;

};


int main(int argc, char **argv) {
  ros::init(argc, argv, "map_listener");
  ros::NodeHandle n;

  ROS_INFO("Map listener node started");
  MapListener map(n);
  
  ros::spin();
  ROS_INFO("Map listener node finished");
  return 0;
}