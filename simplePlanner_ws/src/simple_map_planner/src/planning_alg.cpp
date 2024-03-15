#include "planning_alg.h"


struct Node {
    int id; 
    float x, y; 
    int heuristic, cost; 
    Node* parent;

    Node(int id, float x, float y, int heuristic, int cost, Node *parent) : id(id), x(x), y(y), heuristic(heuristic), cost(cost), parent(parent) {}

    bool operator > (const Node& n) const {
        int h = this->heuristic;
        int c = this->cost;
        return (h+c) > (n.heuristic+n.cost);
    }
};

using NodeQueue = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
using NodeMap = std::map<int, Node>;

PlanningAlg::PlanningAlg() {}

void PlanningAlg::planPath(const geometry_msgs::PoseWithCovarianceStamped& initial_pose, 
                           const geometry_msgs::PoseStamped& goal_pose,
                           const nav_msgs::OccupancyGrid& map) {
    ROS_INFO("planning_alg: planPath"); 
    ROS_INFO("Received map: %d X %d", map.info.width, map.info.height);
    ROS_INFO("Received initial pose: %0.2lf, %0.2lf", 
                                                initial_pose.pose.pose.position.x, 
                                                initial_pose.pose.pose.position.y);
    ROS_INFO("Received goal pose: %0.2lf, %0.2lf", 
                                            goal_pose.pose.position.x, 
                                            goal_pose.pose.position.y);                          
    Node start = Node(0, initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y, 0, 0, nullptr);
    Node goal = Node(1, goal_pose.pose.position.x, goal_pose.pose.position.y, 0, 0, nullptr);
    NodeQueue open_set;
    NodeMap closed_set;
    std::unordered_map<int, int> cost_so_far;

    open_set.push(start);
    cost_so_far[start.id] = 0;

    //algorithm


    
}


