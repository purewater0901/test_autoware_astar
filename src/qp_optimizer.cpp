#include "qp_optimizer.h"

QPOptimizer::QPOptimizer(const OptimizerParam &param)
{
    param_ = param;
    qp_solver_.updateMaxIter(4000);
    qp_solver_.updateRhoInterval(0);  // 0 means automoatic
    qp_solver_.updateEpsRel(1.0e-4);  // def: 1.0e-4
    qp_solver_.updateEpsAbs(1.0e-4);  // def: 1.0e-4
    qp_solver_.updateVerbose(false);
}

void QPOptimizer::setParam(const OptimizerParam &param)
{
    param_ = param;
}

bool QPOptimizer::solve(const double &initial_vel,
                        const double &initial_acc,
                        const std::vector<double> &ref_vel,
                        const std::vector<double> &max_vel)
{
    assert(ref_vel.size()==max_vel.size());
    int N = ref_vel.size();

    /*
     * x = [b0, b1, ..., bN, |  a0, a1, ..., aN, | delta0, delta1, ..., deltaN, | sigma0, sigma1, ..., sigmaN] in R^{4N}
     * b: velocity^2
     * a: acceleration
     * delta: 0 < bi < vmax^2 + delta
     * sigma: amin < ai - sigma < amax
     */

    /*
     * x = [v0, v1, ..., vN]
     * 0 < v < v_ref
     */

    const uint32_t l_variables = N;
    const uint32_t l_constraints = N;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(
            l_constraints, l_variables);  // the matrix size depends on constraint numbers.

    std::vector<double> lower_bound(l_constraints, 0.0);
    std::vector<double> upper_bound(l_constraints, 0.0);

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(l_variables, l_variables);
    std::vector<double> q(l_variables, 0.0);

    /* design objective function */
    for(unsigned int i=0; i<N; ++i)
    {
        P(i, i) = 1.0;
        q[i] = -2*ref_vel[i];
    }

    /*
     * design constraint matrix
     * 0 < v < v_ref^2
     */
    for(unsigned int i=0; i<N; ++i)
    {
        A(i, i) = 1.0;
        upper_bound[i] = ref_vel[i];
        lower_bound[i] = 0.0;
    }

    // initial condition
    const double v0 = initial_vel;
    A(0, 0) = 1.0;  // b0
    upper_bound[0] = v0;
    lower_bound[0] = v0;

    const auto result = qp_solver_.optimize(P, A, q, lower_bound, upper_bound);

    // [x0, x1, ..., xN]
    const std::vector<double> optval = std::get<0>(result);
    for(int i=0; i<N; ++i)
        std::cout << "v_result[" << i << "]: " << optval.at(i) << "[m/s]" << std::endl;


    return true;
}