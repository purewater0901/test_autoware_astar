#ifndef TEST_HYBRID_ASTAR_HASTAR_NODE_H
#define TEST_HYBRID_ASTAR_HASTAR_NODE_H

#include <iostream>
#include <cmath>
#include <vector>

class HAStarNode
{
public:
    HAStarNode(const double& s,
               const double& t,
               const double& v,
               const double& a,
               const int& s_d,
               const int& t_d,
               const int& v_d,
               const double& heuristic_cost,
               const double& goal_s,
               HAStarNode* parent = nullptr);

    HAStarNode(const double& s,
               const double& t,
               const double& v,
               const double& a,
               const int& s_d,
               const int& t_d,
               const int& v_d,
               const double& actual_cost,
               const double& heuristic_cost,
               const double& goal_s,
               HAStarNode* parent = nullptr);

    // getter
    double getScore() { return actual_cost_ + heuristic_cost_; }
    double getPosition(){ return s_; }
    double getTime(){ return t_; }
    double getVelocity(){ return v_; }
    double getAcceleration(){ return a_; }
    double getActualCost() { return actual_cost_; }
    double getHeuristicCost() { return heuristic_cost_; }
    int getPositionIndex(){ return s_d_; }
    int getTimeIndex(){ return t_d_; }
    int getVelocityIndex(){ return v_d_; }
    HAStarNode* getParentNode(){return parent_node_; }

    // setter
    void setActualCost(const double& actual_cost){ actual_cost_ = actual_cost; }
    void setHeuristicCost(const double& heuristic_cost){ heuristic_cost_ = heuristic_cost; }
    void setPosition(const double& s){ s_ = s; }
    void setTime(const double& t){ t_ = t; }
    void setVelocity(const double& v){ v_ = v; }
    void setAcceleration(const double& a){ a_ = a; }
    void setPositionIndex(const int& s_d){ s_d_ = s_d; }
    void setTimeIndex(const int& t_d){ t_d_ = t_d; }
    void setVelocityIndex(const int& v_d){ v_d_ = v_d; }
    void setParentNode(HAStarNode* parent_node){ parent_node_ = parent_node; };


    bool isGoal(const double& goal_s, const double& offset=0.0);

private:
    double s_;
    double t_;
    double v_;
    double a_;
    int s_d_;
    int t_d_;
    int v_d_;
    double actual_cost_;
    double heuristic_cost_;
    HAStarNode* parent_node_;
};

#endif //TEST_HYBRID_ASTAR_HASTAR_NODE_H
