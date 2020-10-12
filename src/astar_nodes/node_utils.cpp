#include <astar_nodes/node_utils.h>

HAStarNode* NodeUtils::findNodeOnList(const std::set<HAStarNode*>& node_list,
                                      const int& s_d,
                                      const int& t_d,
                                      const int& v_d)
{
    for(HAStarNode* node : node_list)
        if(node->getPositionIndex() == s_d &&
           node->getTimeIndex() == t_d &&
           node->getVelocityIndex() == v_d)
            return node;

    return nullptr;
}
