#ifndef PLANNINGALG_H
#define PLANNINGALG_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_map>
#include <utility>
#include <queue>
#include <vector>
#include <functional>
#include <iostream>


// Pre-dichiarazione della struttura Node
struct Node;
struct NodesCompare;

// Utilizzo di forward declaration per i tipi definiti al di fuori della classe
using NodeQueue = std::priority_queue<Node*, std::vector<Node*>, NodesCompare>;
using Coordinate = std::pair<float, float>;
using NodeMap = std::map<Coordinate, Node*>;
using MarkerArray = visualization_msgs::MarkerArray; 

class PlanningAlg {
public:
    PlanningAlg();
    void planPath(const geometry_msgs::PoseWithCovarianceStamped& initial_pose,
                  const geometry_msgs::PoseStamped& goal_pose,
                  const nav_msgs::OccupancyGrid& map);
    
    
private: 
    bool euclidian = true;
    void setMetric(bool boolean);
    float euclidean_heuristic(Node *from, Node *to);
    float manhattan_heuristic(Node *from, Node *to);
    float position_2_grid(float f);
    void push_back_marker(MarkerArray &marker_array, int index, float x, float y, float w, float r, float g, float b);
    void return_path(Node* current, MarkerArray &goal_path, bool euclidian);
    void A_star(int index, bool euclidian, NodeQueue open_set, NodeMap closed_set, MarkerArray goal_path, std::unordered_map<int, int> cost_so_far, Node* start, Node* goal, nav_msgs::OccupancyGrid map);

};

#endif
