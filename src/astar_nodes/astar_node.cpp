#include "astar_nodes/astar_node.h"

AStarNode::AStarNode(const NodeInformation& node_info,
                     const double& heuristic_cost,
                     const double& goal_s,
                     AStarNode* parent)
                     : node_info_(node_info),
                       heuristic_cost_(heuristic_cost), parent_node_(parent)
{
}

AStarNode::AStarNode(const NodeInformation& node_info,
                     const double& actual_cost,
                     const double& heuristic_cost,
                     const double& goal_s,
                     AStarNode* parent)
                     : node_info_(node_info),
                       heuristic_cost_(heuristic_cost),
                       parent_node_(parent),
                       actual_cost_(actual_cost)
{
}

bool AStarNode::isGoal(const double& goal_s, const double& offset)
{
    return goal_s <= node_info_.s + offset;
}

