#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    this->start_node = &model.FindClosestNode(start_x,start_y);
    this->end_node   = &model.FindClosestNode(end_x,end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*(this->end_node));
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
     current_node->FindNeighbors();
     for(auto node : current_node->neighbors){
         node->parent  = current_node;
         node->h_value = CalculateHValue(node);
         node->g_value = current_node->g_value+node->distance(*(current_node));
         node->visited = true;
         this->open_list.push_back(node);
     }
}


RouteModel::Node *RoutePlanner::NextNode() {
   std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node* a, const RouteModel::Node* b){
         return (float)(a->g_value+a->h_value)>(float)(b->g_value+b->h_value);
   });
   RouteModel::Node* lowest_g_h = open_list.back();
   open_list.pop_back();
   return lowest_g_h;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node* it = current_node;
    while(it != this->start_node){
        path_found.push_back(*it);
        distance+=it->distance(*(it->parent));
        it=it->parent;
    }
    path_found.push_back(*(this->start_node));
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale();
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    current_node=this->start_node;
    current_node->visited=true;
    current_node->h_value=CalculateHValue(current_node);
    while(current_node!=this->end_node){
        this->AddNeighbors(current_node);
        current_node=this->NextNode();
    }
    this->m_Model.path=this->ConstructFinalPath(current_node);
}