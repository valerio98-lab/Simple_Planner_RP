#include "planning_alg.h"

using NodeQueue = std::priority_queue<Node*, std::vector<Node*>, std::greater<>>;
using Coordinate = std::pair<float, float>;
using NodeMap = std::map<Coordinate, Node*>;
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

    bool operator > (const Node* n) const {
        return (this->heuristic+this->cost) > (n->heuristic+n->cost);
    }
};

float PlanningAlg::euclidean_heuristic(Node *from, Node *to) {
    return sqrt(std::pow(from->x - to->x,2) + std::pow(from->y - to->y,2));
}

float PlanningAlg::manhattan_heuristic(Node *from, Node *to) {
    return std::abs(from->x - to->x) + std::abs(from->y - to->y);
}

float PlanningAlg::position_2_grid(float f) {
    if(f < 0.f){
        return f = std::floor(f)+0.5;
    }
    else{
        return f = std::floor(f)+0.5;
    }
}

void PlanningAlg::push_back_marker(MarkerArray &marker_array, int index, float x, float y, float w, float r, float g, float b){
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

void PlanningAlg::return_path(Node* current, MarkerArray &goal_path){
    auto it = current;
    for(it; it->parent != nullptr; it=it->parent){
        push_back_marker(goal_path, it->id, it->x, it->y, it->w, 0.0, 0.0, BLUE);
    }
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
    Node* start = new Node(index, position_2_grid(initial_pose.pose.pose.position.x), position_2_grid(initial_pose.pose.pose.position.y), initial_pose.pose.pose.orientation.w, 0, 1.0f, nullptr);
    Node* goal = new Node(index-1, position_2_grid(goal_pose.pose.position.x), position_2_grid(goal_pose.pose.position.y), goal_pose.pose.orientation.w, 0, 1.0f, nullptr);
        
    open_set.push(start);
    cost_so_far[start->id] = 0;
        
    float start_heuristic = euclidean_heuristic(start, goal);
    start->heuristic = start_heuristic;
    start->cost = 0;

    while(!open_set.empty()) {
        Node* current = open_set.top();
        open_set.pop();
        if(current->x == goal->x && current->y == goal->y){
            ROS_INFO("Goal reached!");
            return_path(current, goal_path);
            push_back_marker(goal_path, current->id, current->x, current->y, current->w, RED, 0.0, 0.0);
            marker_array_pub_.publish(goal_path);
            for (auto& pair : closed_set) {
                if (pair.second != current) {
                    delete pair.second;
                }
            }
            while (!open_set.empty()) {
                if (open_set.top() != current) {
                    delete open_set.top();
                }
                open_set.pop();
            }
            return;
        }
        Coordinate current_coord = std::make_pair(current->x, current->y);
        closed_set.insert(std::make_pair(current_coord, current));

        for(int i=-1; i<2; i++){
            for(int j=-1; j<2; j++){
                index += 1;
                if(i==0 && j==0) continue;

                float x_next = current->x + i;
                float y_next = current->y + j;
                if(((x_next) >=0 && x_next < map.info.width) && (y_next >=0 && y_next < map.info.height)) {
                    int index_ = (y_next-0.5) * map.info.width + (x_next-0.5);
                    if(map.data[index_] == 0){
                        Coordinate next_coord = std::make_pair(x_next, y_next);
                        float i_j_cost = (i==0 || j==0) ? 1.0f : sqrt(2.0f);
                        float cost_so_far_next = cost_so_far[current->id] + i_j_cost;
                        
                        if(closed_set.find(next_coord) != closed_set.end()){        //Se già esplorato controlla se è il caso di aggiornare il costo
                            int id_old_node = closed_set[next_coord]->id;
                            float cost_old_node = cost_so_far[id_old_node];
                            if(cost_so_far_next < cost_old_node){
                                closed_set[next_coord]->parent = current;
                                open_set.push(closed_set[next_coord]);
                                closed_set.erase(next_coord);
                            }
                        } 
                        else {                                                      //Altrimenti aggiungi il nodo all'insieme dei nodi da esplorare
                            int id = index;
                            float w_next = 0;
                            Node* next_node = new Node(id, x_next, y_next, w_next, 0, i_j_cost, nullptr);
                            float next_node_heuristic = euclidean_heuristic(next_node, goal); 
                            next_node->heuristic = next_node_heuristic;
                            next_node->parent = current;
                            open_set.push(next_node);
                            cost_so_far[next_node->id] = cost_so_far_next;
                        }
                    }
                }

                
                
                

            }
           
        }
    }
    
}


