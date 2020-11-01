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
                        const std::vector<double>& ref_vel,
                        const std::vector<double>& max_vel,
                        const std::vector<double>& ref_acc,
                        std::vector<double>& qp_time,
                        std::vector<double>& qp_velocity,
                        std::vector<double>& qp_acceleration,
                        std::vector<double>& qp_jerk)
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
     * x = [v[0], v[1], ..., v[N], | a[0], a[1], .... a[N], | jerk[0], jerk[1], ..., jerk[N],| delta[0], ..., delta[N],
     *      | sigma[0], sigma[1], ...., sigma[N], | gamma[0], gamma[1], ..., gamma[N] ]
     * delta: 0 < v[i]-delta[i] < v_ref
     * sigma: amin < a[i] - sigma[i] < amax
     * gamma: jerk_min < jerk[i] - gamma[i] < jerk_max
     */

    const uint32_t l_variables = 6*N;
    const uint32_t l_constraints = 6*N;

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

    // jerk[i]
    for(unsigned int i=2*N; i<3*N; ++i)
        P(i, i) = 10.0;

    // delta[i]
    for(unsigned int i=3*N; i<4*N; ++i)
        P(i, i) = 5.0;

    // sigma[i]
    for(unsigned int i=4*N; i<5*N; ++i)
        P(i, i) = 100000.0;

    // gamma[i]
    for(unsigned int i=5*N; i<6*N; ++i)
        P(i, i) = 100000.0;

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

    /*
     * jerk[i] = (a[i+1]-a[i])/dt
     * jerk[i]*dt - a[i+1] + a[i] = 0.0
     */
    for(int i=2*N+1; i<3*N; ++i)
    {
        int j=i-N-1;
        A(i, j)   = 1.0;    //a[i]
        A(i, j+1) = -1.0;   //a[i+1]
        A(i, i) = param_.dt; //jerk[i]*dt
        upper_bound[i] = 0.0;
        lower_bound[i] = 0.0;
    }

    //initial jerk condition
    A(2*N, 2*N) = 1.0; //jerk[0]
    upper_bound[2*N] = 0.0;
    lower_bound[2*N] = 0.0;

    /*
     * Maximum Velocity Constraint
     * 0 < v[i] - delta[i] < v_ref[i]
     */
    for(int i=3*N; i<4*N; ++i)
    {
        int j = i - 3*N;
        A(i, j) = 1.0; //v[i]
        A(i, i) = -1.0; //-delta[i]
        upper_bound[i] = ref_vel[i-3*N];
        lower_bound[i] = 0.0;
    }

    /*
     * Acceleration Constraint
     * amin < a[i] - sigma[i] < amax
     */
    for(int i=4*N; i<5*N; ++i)
    {
        int j = i - 3*N;
        A(i, j) = 1.0; //a[i]
        A(i, i) = -1.0; //-sigma[i]
        upper_bound[i] = param_.max_accel;
        lower_bound[i] = param_.min_decel;
    }

    /*
     * Jerk Constraint
     * jerk_min < jerk[i] - gamma[i] < jerk_max
     */
    for(int i=5*N; i<6*N; ++i)
    {
        int j = i - 3*N;
        A(i, j) = 1.0; //jerk[i]
        A(i, i) = -1.0; //-gamma[i]
        upper_bound[i] = param_.max_jerk;
        lower_bound[i] = param_.min_jerk;
    }


    const auto result = qp_solver_.optimize(P, A, q, lower_bound, upper_bound);

    // [x0, x1, ..., xN]
    const std::vector<double> optval = std::get<0>(result);
    for(int i=0; i<N; ++i)
        std::cout << "v_result[" << i << "]: " << optval.at(i) << "[m/s]     " << ref_vel[i] << "[m/s]" << std::endl;
    std::cout << "---------------------" << std::endl;
    for(int i=N; i<2*N; ++i)
        std::cout << "a_result[" << i-N << "]: " << optval.at(i) << "[m/s^2]  " << ref_acc[i-N] << "[m/s^2]" << std::endl;

    qp_time.resize(N);
    qp_velocity.resize(N);
    qp_acceleration.resize(N);
    qp_jerk.resize(N);
    qp_velocity[0] = 0.0;
    for(int i=1; i<N; ++i)
        qp_time[i] = qp_time[i-1] + param_.dt;
    for(int i=0; i<N; ++i)
        qp_velocity[i] = optval.at(i);
    for(int i=N; i<2*N; ++i)
        qp_acceleration[i] = optval.at(i);
    for(int i=2*N; i<3*N; ++i)
        qp_jerk[i] = optval.at(i);

    return true;
}