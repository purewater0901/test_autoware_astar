#include "qp_optimizer.h"

QPOptimizer::QPOptimizer(const OptimizerParam &param) : param_(param)
{
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
     * x = [v[0], v[1], ..., v[N], | a[0], a[1], .... a[N]]
     * 0 < v < v_ref
     */

    const uint32_t l_variables = 2*N;
    const uint32_t l_constraints = 2*N;

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

    // initial velocity condition
    const double v0 = initial_vel;
    A(0, 0) = 1.0;  // b0
    upper_bound[0] = v0;
    lower_bound[0] = v0;

    /*
     * a[i] = (v[i+1]-v[i])/dt
     * a[i]*dt - v[i+1] + v[i] = 0.0
     */
    for(int i=N+1; i<2*N; ++i)
    {
        int j=i-N-1;
        A(i, j)   = 1.0; //v[i]
        A(i, j+1) = -1.0;  //v[i+1]
        A(i, i) = param_.dt; //a[i]*dt
        upper_bound[i] = 0.0;
        lower_bound[i] = 0.0;
    }

    //initial acceleration condition
    A(N, N) = 1.0; //a[0]
    upper_bound[N] = initial_acc;
    lower_bound[N] = initial_acc;

    const auto result = qp_solver_.optimize(P, A, q, lower_bound, upper_bound);

    // [x0, x1, ..., xN]
    const std::vector<double> optval = std::get<0>(result);
    for(int i=0; i<N; ++i)
        std::cout << "v_result[" << i << "]: " << optval.at(i) << "[m/s]" << std::endl;
    std::cout << "---------------------" << std::endl;
    for(int i=N; i<2*N-1; ++i)
        std::cout << "a_result[" << i-N << "]: " << optval.at(i) << "[m/s^2]" << std::endl;


    return true;
}