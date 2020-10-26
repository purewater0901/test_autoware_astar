#ifndef TEST_HYBRID_ASTAR_QP_OPTIMIZER_H
#define TEST_HYBRID_ASTAR_QP_OPTIMIZER_H

#include <Eigen/Eigen>
#include <vector>
#include <cmath>
#include <cassert>
#include <iostream>
#include "solver_interface/osqp_interface.h"
#include "osqp.h"

class QPOptimizer
{
public:
    struct OptimizerParam
    {
        double max_accel;
        double min_decel;
        double max_jerk;
        double min_jerk;
        double jerk_weight;
        double over_v_weight;
        double over_a_weight;
    };

    QPOptimizer(const OptimizerParam& param);

    void setParam(const OptimizerParam& param);

    bool solve(const double& initial_vel,
               const double& initial_acc,
               const std::vector<double>& ref_vel,
               const std::vector<double>& max_vel);

private:
    OptimizerParam param_;
    osqp::OSQPInterface qp_solver_;
};

#endif //TEST_HYBRID_ASTAR_QP_OPTIMIZER_H
