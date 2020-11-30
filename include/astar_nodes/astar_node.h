#ifndef TEST_HYBRID_ASTAR_ASTAR_NODE_H
#define TEST_HYBRID_ASTAR_ASTAR_NODE_H

#include <iostream>
#include <cmath>
#include <vector>

struct NodeInformation
{
    double s; // Position
    double v; // Velocity
    double t; // Time
    double a; // Acceleration
    double max_v; // Maximum Velocity Index Constraint at position s_id
    int s_id; // Position Id
    int v_id; // Velocity Id
    int t_id; // Time Id
};

class AStarNode
{
public:
    AStarNode(const NodeInformation& node_info,
              const double& heuristic_cost,
              const double& goal_s,
              AStarNode* parent = nullptr);

    AStarNode(const NodeInformation& node_info,
              const double& actual_cost,
              const double& heuristic_cost,
              const double& goal_s,
              AStarNode* parent = nullptr);

    // getter
    // getter
    double getScore() { return actual_cost_ + heuristic_cost_; }
    double getPosition(){ return node_info_.s; }
    double getTime(){ return node_info_.t; }
    double getVelocity(){ return node_info_.v; }
    double getAcceleration(){ return node_info_.a; }
    double getActualCost() { return actual_cost_; }
    double getHeuristicCost() { return heuristic_cost_; }
    int getPositionIndex(){ return node_info_.s_id; }
    int getTimeIndex(){ return node_info_.t_id; }
    int getVelocityIndex(){ return node_info_.v_id; }
    AStarNode* getParentNode(){return parent_node_; }

    // setter
    // setter
    void setActualCost(const double& actual_cost){ actual_cost_ = actual_cost; }
    void setHeuristicCost(const double& heuristic_cost){ heuristic_cost_ = heuristic_cost; }
    void setPosition(const double& s){ node_info_.s = s; }
    void setTime(const double& t){ node_info_.t = t; }
    void setVelocity(const double& v){ node_info_.v = v; }
    void setAcceleration(const double& a){ node_info_.a = a; }
    void setPositionIndex(const int& s_id){ node_info_.s_id = s_id; }
    void setTimeIndex(const int& t_id){ node_info_.t_id = t_id; }
    void setVelocityIndex(const int& v_id){ node_info_.v_id = v_id; }
    void setParentNode(AStarNode* parent_node){ parent_node_ = parent_node; }


    bool isGoal(const double& goal_s, const double& offset=0.0);

private:
    NodeInformation node_info_;
    double actual_cost_;
    double heuristic_cost_;
    AStarNode* parent_node_;
};

#endif //TEST_HYBRID_ASTAR_ASTAR_NODE_H
