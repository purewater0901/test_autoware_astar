#ifndef TEST_HYBRID_ASTAR_ASTAR_OPTIMIZER_H
#define TEST_HYBRID_ASTAR_ASTAR_OPTIMIZER_H

#include <numeric>
#include <vector>
#include <set>
#include <chrono>
#include <eigen3/Eigen/Core>

#include <astar_nodes/hastar_node.h>
#include <astar_nodes/node_utils.h>

class AStarOptimizer
{
public:
    AStarOptimizer() = default;

    bool solve(const double initial_vel,
               const double initial_acc,
               const int N,
               const double goal_s,
               const std::vector<double>& output_trajectory_index,
               const std::vector<double>& interpolated_max_velocity);

    bool calculateByFixDistance(const double& initial_vel,
                                const double& initial_acc,
                                const unsigned int& N,
                                const double& ds,
                                const double& dt,
                                const double& dv,
                                const double& goal_s,
                                const double& offset,
                                const double& tol,
                                const double& t_max,
                                const double& weight_v,
                                const double& weight_a,
                                const std::vector<double>& output_trajectory_index,
                                const std::vector<double>& interpolated_max_velocity,
                                const std::vector<double>& da_list);

    double calculateActualCost(const double& v, const double& reference_v,
                               const double& weight_v, const double& current_a, const double& next_a, const double& dt,
                               const double& weight_a, const double& offset);

    double calculateHeuristicCost(const double& s, const double& goal_s);

    double calculateMaximumVelocity(const int& index, const std::vector<double>& vmax_vec);

    void createNewNode(std::set<HAStarNode*>& open_node_list,
                       std::set<HAStarNode*>& closed_node_list,
                       HAStarNode* current_node,
                       const double& weight_v, const double& weight_a, const double& goal_s,
                       const double& next_s, const double& next_v, const double& next_t,
                       const double& next_max_v, const double& a_command,
                       const int& next_s_id, const int& next_v_id, const int& next_t_id);
};

#endif //TEST_HYBRID_ASTAR_ASTAR_OPTIMIZER_H
