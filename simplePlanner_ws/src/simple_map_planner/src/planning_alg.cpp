#include "planning_alg.h"


struct Node {
    int id; 
    float x, y; 
    float heuristic, cost; 
    Node* parent;

    Node(int id, float x, float y, float heuristic, float cost, Node *parent) : id(id), x(x), y(y), heuristic(heuristic), cost(cost), parent(parent) {}

    bool operator > (const Node& n) const {
        return (this->heuristic+this->cost) > (n.heuristic+n.cost);
    }
};

float euclidean_heuristic(Node& from, Node& to) {
    return sqrt(std::pow(from.x - to.x,2) + std::pow(from.y - to.y,2));
}

float manhattan_heuristic(Node& from, Node& to) {
    return std::abs(from.x - to.x) + std::abs(from.y - to.y);
}

float position_2_grid(float f) {
    if(f < 0.f){
        return f = std::floor(f)+0.5f;
    }
    else{
        return f = std::floor(f)+0.5f;
    }
}

using NodeQueue = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
using NodeMap = std::map<int, Node>;
ros::Publisher marker_array_pub_;

PlanningAlg::PlanningAlg() {
    ros::NodeHandle nh;
    marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000, true);
}

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

    Node start = Node(0, position_2_grid(initial_pose.pose.pose.position.x), position_2_grid(initial_pose.pose.pose.position.y), 0, 0, nullptr);
    Node goal = Node(1, position_2_grid(goal_pose.pose.position.x), position_2_grid(goal_pose.pose.position.y), 0, 0, nullptr);
    NodeQueue open_set;
    NodeMap closed_set;
    std::unordered_map<int, int> cost_so_far;
    
    // open_set.push(start);
    // cost_so_far[start.id] = 0;

    
    // float start_heuristic = euclidean_heuristic(start, goal);
    // start.heuristic = start_heuristic;

    // while(!open_set.empty()) {
    //     Node current = open_set.top();
    //     open_set.pop();
    //     if()
    // }



    
}


