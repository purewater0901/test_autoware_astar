#ifndef TEST_HYBRID_ASTAR_NODE_UTILS_H
#define TEST_HYBRID_ASTAR_NODE_UTILS_H

#include <iostream>
#include <set>
#include <astar_nodes/hastar_node.h>
#include <astar_nodes/astar_node.h>

namespace NodeUtils
{
    HAStarNode* findNodeOnList(const std::set<HAStarNode*>& node_list,
                               const int& s_d,
                               const int& t_d,
                               const int& v_d);

    AStarNode* findNodeOnList(const std::set<AStarNode*>& node_list,
                              const int& s_d,
                              const int& t_d,
                              const int& v_d);
}

#endif //TEST_HYBRID_ASTAR_NODE_UTILS_H
