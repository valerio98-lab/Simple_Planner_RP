#include "planning_alg.h"

using NodeQueue = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
using NodeMap = std::map<int, Node>;
using MarkerArray = visualization_msgs::MarkerArray; 
ros::Publisher marker_array_pub_;

#define RED 1.0
#define GREEN 1.0
#define BLUE 1.0

PlanningAlg::PlanningAlg() {
    ros::NodeHandle nh;
    marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000, true);
}

struct Node {
    int id;
    float x, y, w;
    float heuristic, cost; 
    Node* parent;

    Node(int id, float x, float y, float w, float heuristic, float cost, Node *parent) : id(id), x(x), y(y), w(w), heuristic(heuristic), cost(cost), parent(parent) {}

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

void push_back_marker(MarkerArray &marker_array, int index, float x, float y, float w, float r, float g, float b){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "poses";
    marker.id = index; 
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.w = w;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker_array.markers.push_back(marker);
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
    NodeQueue open_set;
    NodeMap closed_set;
    MarkerArray goal_path;
    std::unordered_map<int, int> cost_so_far;

    int index = 0;
    Node start = Node(index, position_2_grid(initial_pose.pose.pose.position.x), position_2_grid(initial_pose.pose.pose.position.y), initial_pose.pose.pose.orientation.w, 0, 1.0f, nullptr);
    Node goal = Node(index-1, position_2_grid(goal_pose.pose.position.x), position_2_grid(goal_pose.pose.position.y), goal_pose.pose.orientation.w, 0, 1.0f, nullptr);
        
    open_set.push(start);
    cost_so_far[start.id] = 0;
        
    float start_heuristic = euclidean_heuristic(start, goal);
    start.heuristic = start_heuristic;
    start.cost = 0;

    push_back_marker(goal_path, start.id, start.x, start.y, goal.w, 0.0, GREEN, 0.0);
    push_back_marker(goal_path, goal.id, goal.x, goal.y, goal.w, RED, 0.0, 0.0);
    marker_array_pub_.publish(goal_path);

    while(!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();
        if(current.x == goal.x && current.y == goal.y){
            ROS_INFO("Goal reached!");
            push_back_marker(goal_path, current.id, current.x, current.y, current.w, 0.0, 0.0, BLUE);
            marker_array_pub_.publish(goal_path);
            return 1;
        }
        closed_set[current.id] = current;
        for(int i=-1; i<8; i++){
            for(int j=-1; j<2; j++){
                index += 1;
                if(!i==0 && !j==0){
                    /*TODO: 
                    Controllare se la cella è occupata. 
                    Controllare se la cella è già stata visitata.         
                    */     
                    float x_next = current.x + i;
                    float y_next = current.y + j;
                    int id = index;
                    float w_next = 0;
                    Node next_node = Node(id, x_next, y_next, w_next, 0, 1.0f, nullptr);
                    float next_node_cost = cost_so_far[current.id] + next_node.cost;
                    float next_node_heuristic = euclidean_heuristic(next_node, goal); 
                    next_node.heuristic = next_node_heuristic;
                    next_node.parent = &current;
                    cost_so_far[next_node.id].insert(next_node_cost);
                    open_set.push(next_node);
                }
            }
           
        }
    }
    
}


