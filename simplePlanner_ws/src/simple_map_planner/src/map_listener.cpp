
#include <ros/ros.h> 
#include <nav_msgs/OccupancyGrid.h> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class MapListener {
  public:
    MapListener(ros::NodeHandle& n) {
      n.subscribe("map", 1000, &MapListener::mapCallback, this);
      n.subscribe("initialpose", 1000, &MapListener::initialPoseCallback, this);
      n.subscribe("move_base_simple/goal", 1000, &MapListener::goalPoseCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
      map_received = true;
      ROS_INFO("Received map: %d X %d", msg->info.width, msg->info.height);
    }

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
      initial_pose = *msg;
      initial_pose_received = true;
      ROS_INFO("Received initial pose: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", 
                                                initial_pose.pose.pose.position.x, 
                                                initial_pose.pose.pose.position.y, 
                                                initial_pose.pose.pose.position.z, 
                                                initial_pose.pose.pose.orientation.x, 
                                                initial_pose.pose.pose.orientation.y, 
                                                initial_pose.pose.pose.orientation.z, 
                                                initial_pose.pose.pose.orientation.w); 
    }

    void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
      goal_pose = *msg;
      goal_pose_received = true;
      ROS_INFO("Received goal pose: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", 
                                            goal_pose.pose.position.x, 
                                            goal_pose.pose.position.y, 
                                            goal_pose.pose.position.z, 
                                            goal_pose.pose.orientation.x, 
                                            goal_pose.pose.orientation.y, 
                                            goal_pose.pose.orientation.z, 
                                            goal_pose.pose.orientation.w);

    }
  private:
    geometry_msgs::PoseStamped goal_pose;
    geometry_msgs::PoseWithCovarianceStamped initial_pose;
    bool map_received = false;
    bool initial_pose_received = false;
    bool goal_pose_received = false;

};


int main(int argc, char **argv) {
  ros::init(argc, argv, "map_listener");
  ros::NodeHandle n;

  MapListener map(n);


  ros::spin();
  
  return 0;
}