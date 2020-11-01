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

namespace Utils
{
    void outputResultToFile(const std::string& filename,
                            const std::vector<double>& qp_time,
                            const std::vector<double>& qp_velocity,
                            const std::vector<double>& qp_acceleration,
                            const std::vector<double>& qp_jerk);

    void outputResultToFile(const std::vector<double>& data);
}

#endif //TEST_HYBRID_ASTAR_UTILS_H
