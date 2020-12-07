#ifndef TEST_HYBRID_ASTAR_ASTAR_OPTIMIZER_H
#define TEST_HYBRID_ASTAR_ASTAR_OPTIMIZER_H

#include <numeric>
#include <vector>
#include <set>
#include <chrono>
#include <Eigen/Eigen>

#include <astar_nodes/astar_node.h>
#include <astar_nodes/node_utils.h>

class AStarOptimizer
{
public:
    struct AStarOptimizerParam
    {
        double max_accel; // Maximum Acceleration
        double min_decel; // Minimum Acceleration
        double weight_s; // progress weight
        double weight_v; // velocity weight
        double weight_jerk; // jerk weight
        double weight_over_v; // weight for over velocity
        double dt;       // time interval
        double ds;       // position interval
        double dv;       // velocity interval
        double max_time; // maximum time
    };

    struct AStarOutputInfo
    {
        std::vector<double> optimum_position;
        std::vector<double> optimum_velocity;
        std::vector<double> optimum_acceleration;
        std::vector<double> optimum_time;

        void push_back(AStarNode* node)
        {
            optimum_position.push_back(node->getPosition());
            optimum_velocity.push_back(node->getVelocity());
            optimum_acceleration.push_back(node->getAcceleration());
            optimum_time.push_back(node->getTime());
        }

        void reserve(const unsigned int& N)
        {
            optimum_position.reserve(N);
            optimum_velocity.reserve(N);
            optimum_acceleration.reserve(N);
            optimum_time.reserve(N);
        }

        void reverse()
        {
            std::reverse(optimum_position.begin(), optimum_position.end());
            std::reverse(optimum_velocity.begin(), optimum_velocity.end());
            std::reverse(optimum_acceleration.begin(), optimum_acceleration.end());
            std::reverse(optimum_time.begin(), optimum_time.end());
        }
    };

    explicit AStarOptimizer(const AStarOptimizerParam& param);

    bool solve(
            const double initial_vel, const double initial_acc, const int closest,
            const std::vector<double>& vref,
            const std::vector<double>& vmax,
            const std::vector<double>& input_trajectory_index,
            AStarOutputInfo& output_info);



    bool calculateByFixDistance(const double& initial_vel, const double& initial_acc, const double& goal_s,
                                const std::vector<double>& output_trajectory_index,
                                const std::vector<double>& ref_velocity,
                                const std::vector<double>& max_velocity,
                                const std::vector<std::pair<int, int>>& directions,
                                AStarOutputInfo& output_info);

    double calculateActualCost(const double& current_v,
                               const double& reference_v,
                               const double& current_a,
                               const double& next_a,
                               const double& dt,
                               const double& offset);

    double calculateHeuristicCost(const double& s, const double& goal_s);

    double calculateMaximumVelocity(const int& index, const std::vector<double>& vmax_vec);

    void createNewNode(std::set<AStarNode*>& open_node_list,
                       std::set<AStarNode*>& closed_node_list,
                       AStarNode* current_node,
                       const double& goal_s,
                       const NodeInformation& node_info);

private:
    AStarOptimizerParam param_;

};

#endif //TEST_HYBRID_ASTAR_ASTAR_OPTIMIZER_H
