#include <astar_nodes/hastar_node.h>

HAStarNode::HAStarNode(const double& s,
                       const double& t,
                       const double& v,
                       const double& a,
                       const int& s_d,
                       const int& t_d,
                       const int& v_d,
                       const double& heuristic_cost,
                       const double& goal_s,
                       HAStarNode * parent)
        : s_(s), t_(t), v_(v), a_(a), s_d_(s_d), t_d_(t_d), v_d_(v_d),
          heuristic_cost_(heuristic_cost), parent_node_(parent)
{
}

HAStarNode::HAStarNode(const double& s,
                       const double& t,
                       const double& v,
                       const double& a,
                       const int& s_d,
                       const int& t_d,
                       const int& v_d,
                       const double& actual_cost,
                       const double& heuristic_cost,
                       const double& goal_s,
                       HAStarNode* parent)
        : s_(s), t_(t), v_(v), a_(a), s_d_(s_d), t_d_(t_d), v_d_(v_d),
          actual_cost_(actual_cost), heuristic_cost_(heuristic_cost),
          parent_node_(parent)
{
}

bool HAStarNode::isGoal(const double& goal_s, const double& offset)
{
    return goal_s <= s_ + offset;
}