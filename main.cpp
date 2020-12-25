#include <iostream>
#include <vector>

#include "astar_optimizer.h"
#include "qp_optimizer.h"
#include "interpolate.h"
#include "utils.h"

int main() {

    const int N = 55;
    const double initial_vel = 5.0;
    const double initial_acc = 1.0;
    const double ds = 1.0;
    const double jerk_acc = 0.8;
    const double max_accel = 1.0;

    AStarOptimizer::AStarOptimizerParam astar_param{};
    astar_param.max_accel = 1.0;
    astar_param.min_decel = -1.0;
    astar_param.weight_s = 10.0;
    astar_param.weight_v = 100.0;
    astar_param.weight_jerk = 1.0;
    astar_param.weight_over_v = 1e6;
    astar_param.dt = 1.0;
    astar_param.ds = ds;
    astar_param.dv = 1.0;
    astar_param.max_time = 100.0;

    std::vector<double> s_longitudinal(N, 0.0);
    std::vector<double> v_longitudinal(N, 0.0);

    // position
    for(int i=0; i<N; ++i)
        s_longitudinal[i] = i*ds;

    for(int i=0; i<30; ++i)
        v_longitudinal[i] = 10.0;
    for(int i=30; i<N; ++i)
        v_longitudinal[i] = 16.0;
    v_longitudinal.back() = 0.0;

    // 2. Forward Filter
    std::vector<double> filtered_velocity(v_longitudinal.size());
    filtered_velocity.front() = initial_vel;
    double current_vel = initial_vel;
    double current_acc = initial_acc;
    for(int i=1; i<N; ++i)
    {
        double dt = 0.0;
        if(std::fabs(current_vel)<1e-6)
            dt = sqrt(2*ds/max_accel);
        else
            dt = ds/current_vel;

        current_acc = std::max(current_acc + jerk_acc*dt, max_accel);
        current_vel = std::min(current_vel + current_acc * dt, v_longitudinal[i]);

        // Store Filtered Velocity
        filtered_velocity[i] = current_vel;
    }

    //3. Backward Filter
    filtered_velocity.back() = v_longitudinal.back();
    current_vel = v_longitudinal.back();
    current_acc = 0.0;
    for(int i=N-2; i>=0; --i)
    {
        double dt;
        if(std::fabs(current_vel)<1e-4)
            dt = sqrt(2*ds/max_accel);
        else
            dt = ds/current_vel;

        current_acc = std::max(current_acc + jerk_acc*dt, max_accel);
        current_vel = std::min(current_vel + current_acc * dt, filtered_velocity[i]);

        // Store Filtered Velocity
        filtered_velocity[i] = current_vel;
    }

    for(int i=0; i<v_longitudinal.size(); ++i)
        std::cout << "v[" << i << "]: " << v_longitudinal[i] << " Filtered Velocity: " << filtered_velocity[i] << std::endl;


    AStarOptimizer astar_optimizer(astar_param);
    AStarOptimizer::AStarOutputInfo astar_output_info;
    std::chrono::system_clock::time_point  start, end;
    start = std::chrono::system_clock::now();
    astar_optimizer.solve(initial_vel, initial_acc, 0, filtered_velocity, v_longitudinal, s_longitudinal,
                            astar_output_info);
    end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    std::cout << "total time: " << elapsed << "[ms]" << std::endl;

    /*
    std::string astar_filename = "../result/astar_result.csv";
    Utils::outputResultToFile(astar_filename,astar_output_info);
     */
    double qp_dt = 0.2;
    double max_time = astar_output_info.optimum_time.back();
    int qp_size = static_cast<int>(max_time/qp_dt);
    std::vector<double> input_time(qp_size, 0.0);
    for(int i=1; i<qp_size; ++i)
        input_time[i] = input_time[i-1] + qp_dt;

    std::vector<double> input_velocity;
    std::vector<double> input_acceleration;
    LinearInterpolate::interpolate(astar_output_info.optimum_time, astar_output_info.optimum_velocity, input_time, input_velocity);
    LinearInterpolate::interpolate(astar_output_info.optimum_time, astar_output_info.optimum_acceleration, input_time, input_acceleration);
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
    qp_optimizer.solve(initial_vel, initial_acc, input_velocity, input_velocity, input_acceleration,
                       qp_time, qp_velocity, qp_acceleration, qp_jerk);

    std::string qp_filename = "../result/qp_result.csv";
    std::string astar_filename = "../result/astar_result.csv";
    std::string velocity_filename = "../result/reference_velocity.csv";
    Utils::outputVelocityToFile(velocity_filename, s_longitudinal, v_longitudinal, filtered_velocity);
    Utils::outputResultToFile(qp_filename, qp_time, qp_velocity, qp_acceleration, qp_jerk, qp_dt);
    Utils::outputResultToFile(astar_filename,astar_output_info);

    return 0;
}
