#ifndef PLANNINGALG_H
#define PLANNINGALG_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <unordered_map>
#include <queue>
#include <vector>
#include <functional>


// Pre-dichiarazione della struttura Node
struct Node;

// Utilizzo di forward declaration per i tipi definiti al di fuori della classe
using NodeQueue = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
using NodeMap = std::map<int, Node>;

class PlanningAlg {
public:
    PlanningAlg();
    void planPath(const geometry_msgs::PoseWithCovarianceStamped& initial_pose,
                  const geometry_msgs::PoseStamped& goal_pose,
                  const nav_msgs::OccupancyGrid& map);

};

#endif
