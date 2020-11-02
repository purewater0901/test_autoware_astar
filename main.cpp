#include <iostream>
#include <vector>

#include "astar_optimizer.h"
#include "qp_optimizer.h"
#include "interpolate.h"
#include "utils.h"

int main() {

    //int N = 59;
    //double initial_velocity = 7.94965;
    //double initial_acceleration = -1.94025;
    int N = 55;
    //double initial_velocity = 8.13652;
    double initial_velocity = 7.0;
    double initial_acceleration = -0.8;

    std::vector<double> s_longitudinal(N, 0.0);
    std::vector<double> v_longitudinal(N, 0.0);

    // position
    for(int i=0; i<N; ++i)
        s_longitudinal[i] = i*1.0;

    /*
    v_longitudinal[0] = 13.8899;
    v_longitudinal[1] = 13.201;
    v_longitudinal[2] = 11.8409;
    v_longitudinal[3] = 11.4622;
    v_longitudinal[4] = 11.4622;
    v_longitudinal[5] = 11.4479;
    v_longitudinal[6] = 8.5739;
    v_longitudinal[7] = 5.2231;
    v_longitudinal[8] = 3.75684;
    v_longitudinal[9] = 2.99691;
    v_longitudinal[10] = 2.74022;
    for(int i=11; i<=28; ++i)
        v_longitudinal[i] = 2.74;
    v_longitudinal[29] = 2.80168;
    v_longitudinal[30] = 3.02571;
    v_longitudinal[31] = 3.33942;
    v_longitudinal[32] = 3.7629;
    v_longitudinal[33] = 4.3942;
    v_longitudinal[34] = 5.52877;
    v_longitudinal[35] = 7.6872;
    v_longitudinal[36] = 8.94736;
    v_longitudinal[37] = 8.94736;
    v_longitudinal[38] = 12.0841;
    v_longitudinal[39] = 13.7873;
    for(int i=40; i<=57; ++i)
        v_longitudinal[i] = 13.8889;
    v_longitudinal[58] = 0.0;

    double s_goal = 57.64;
     */

    v_longitudinal[0] = 11.661;
    v_longitudinal[1] = 9.37006;
    v_longitudinal[2] = 8.21379;
    v_longitudinal[3] = 8.18014;
    v_longitudinal[4] = 7.66211;
    v_longitudinal[5] = 5.67608;
    v_longitudinal[6] = 3.85434;
    v_longitudinal[7] = 3.0538;
    v_longitudinal[8] = 2.76943;
    for(int i=9; i<=26; ++i)
        v_longitudinal[i] = 2.74;
    v_longitudinal[27] = 2.81086;
    v_longitudinal[28] = 3.00792;
    v_longitudinal[29] = 3.28135;
    v_longitudinal[30] = 3.63436;
    v_longitudinal[31] = 4.14298;
    v_longitudinal[32] = 4.98283;
    v_longitudinal[33] = 6.68678;
    v_longitudinal[34] = 8.53884;
    v_longitudinal[35] = 8.53884;
    v_longitudinal[36] = 8.94713;
    v_longitudinal[37] = 12.3301;
    for(int i=38; i<=41; ++i)
        v_longitudinal[i] = 13.8889;
    v_longitudinal[42] = 13.8845;
    v_longitudinal[43] = 13.6195;
    v_longitudinal[44] = 12.6215;
    v_longitudinal[45] = 11.7284;
    v_longitudinal[46] = 11.0689;
    v_longitudinal[47] = 10.8736;
    for(int i=48; i<=51; ++i)
        v_longitudinal[i] = 10.8695;
    v_longitudinal[52] = 10.92;
    v_longitudinal[53] = 11.2283;
    v_longitudinal[54] = 0.0;

    double s_goal = 53.8351;

    for(int i=0; i<v_longitudinal.size(); ++i)
        std::cout << "v[" << i << "]: " << v_longitudinal[i] << std::endl;

    AStarOptimizer optimizer;
    std::vector<double> optimum_position;
    std::vector<double> optimum_velocity;
    std::vector<double> optimum_acceleration;
    std::vector<double> optimum_time;
    optimizer.solve(initial_velocity, initial_acceleration, N, s_goal, s_longitudinal, v_longitudinal,
                    optimum_position, optimum_velocity, optimum_acceleration, optimum_time);

    double qp_dt = 0.2;
    double max_time = optimum_time.back();
    int qp_size = static_cast<int>(max_time/qp_dt);
    std::vector<double> input_time(qp_size, 0.0);
    for(int i=1; i<qp_size; ++i)
        input_time[i] = input_time[i-1] + qp_dt;

    std::vector<double> input_velocity;
    std::vector<double> input_acceleration;
    LinearInterpolate::interpolate(optimum_time, optimum_velocity, input_time, input_velocity);
    LinearInterpolate::interpolate(optimum_time, optimum_acceleration, input_time, input_acceleration);
    input_velocity.back() = 0.0;

    QPOptimizer::OptimizerParam param{};
    param.max_accel = 1.0;
    param.min_decel = -1.0;
    param.max_jerk = 0.3;
    param.min_jerk = -0.3;
    param.jerk_weight = 1.0;
    param.over_a_weight = 1.0;
    param.over_v_weight = 1.0;
    param.dt = qp_dt;
    QPOptimizer qp_optimizer(param);

    std::vector<double> qp_time;
    std::vector<double> qp_velocity;
    std::vector<double> qp_acceleration;
    std::vector<double> qp_jerk;
    qp_optimizer.solve(initial_velocity, initial_acceleration, input_velocity, input_velocity, input_acceleration,
                       qp_time, qp_velocity, qp_acceleration, qp_jerk);

    std::string qp_filename = "../result/qp_result.csv";
    std::string astar_filename = "../result/astar_result.csv";
    Utils::outputResultToFile(qp_filename, qp_time, qp_velocity, qp_acceleration, qp_jerk);
    Utils::outputResultToFile(astar_filename, input_time, input_velocity, input_acceleration);

    return 0;
}
