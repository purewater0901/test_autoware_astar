#include <astar_nodes/node_utils.h>
#include <algorithm>

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


AStarNode* NodeUtils::findNodeOnList(const std::set<AStarNode*>& node_list,
                                     const int& s_d,
                                     const int& t_d,
                                     const int& v_d)
{
    auto result = std::find_if(node_list.begin(), node_list.end(), [&](AStarNode* node){return (node->getPositionIndex() == s_d &&
                                                                                        node->getTimeIndex() == t_d &&
                                                                                        node->getVelocityIndex() == v_d);});
    if(result==node_list.end())
        return nullptr;
    else
        return *result;
    /*
    for(AStarNode* node : node_list)
        if(node->getPositionIndex() == s_d &&
           node->getTimeIndex() == t_d &&
           node->getVelocityIndex() == v_d)
            return node;
    return nullptr;
            */
}