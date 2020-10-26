#ifndef TEST_HYBRID_ASTAR_QP_OPTIMIZER_H
#define TEST_HYBRID_ASTAR_QP_OPTIMIZER_H

#include <Eigen/Eigen>
#include <vector>
#include <cmath>
#include "osqp_interface.h"
#include "osqp.h"

class QPOptimizer
{
public:
    struct OptimizerParam
    {
        double max_accel;
        double min_decel;
        double jerk_weight;
        double over_v_weight;
        double over_a_weight;
    };

    QPOptimizer(const OptimizerParam& param);

private:
    OptimizerParam param_;
    osqp::OSQPInterface qp_solver_;
};

#endif //TEST_HYBRID_ASTAR_QP_OPTIMIZER_H
