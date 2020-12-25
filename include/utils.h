#ifndef TEST_HYBRID_ASTAR_UTILS_H
#define TEST_HYBRID_ASTAR_UTILS_H

#include <iostream>
#include <memory>
#include <Eigen/Eigen>
#include <vector>
#include <chrono>
#include <string>
#include <fstream>
#include <cassert>
#include "astar_optimizer.h"

namespace Utils
{
    void outputVelocityToFile(const std::string& filename,
                              const std::vector<double>& position,
                              const std::vector<double>& original_velocity,
                              const std::vector<double>& filtered_velocity);

    void outputResultToFile(const std::string& filename,
                            const std::vector<double>& qp_time,
                            const std::vector<double>& qp_velocity,
                            const std::vector<double>& qp_acceleration,
                            const std::vector<double>& qp_jerk,
                            const double& dt);

    void outputResultToFile(const std::string& filename,
                            const AStarOptimizer::AStarOutputInfo& output_info);

}

#endif //TEST_HYBRID_ASTAR_UTILS_H
