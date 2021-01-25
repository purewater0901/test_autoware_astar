#include "utils.h"

namespace Utils
{
    void outputVelocityToFile(const std::string& filename,
                              const std::vector<double>& position,
                              const std::vector<double>& original_velocity,
                              const std::vector<double>& filtered_velocity,
                              const std::vector<double>& filtered_acc)
{
    assert(original_velocity.size() == filtered_velocity.size());
        std::ofstream writing_file;
        writing_file.open(filename, std::ios::out);
        writing_file << "position" << "," << "original_velocity" << "," << "filtered_velocity" << ","  << "filtered_acc" << std::endl;
        for(int i=0; i<original_velocity.size(); ++i)
            writing_file << position[i] << "," << original_velocity[i] << "," << filtered_velocity[i] << "," << filtered_acc[i] << std::endl;

        writing_file.close();
    }

    void outputResultToFile(const std::string& filename,
                            const std::vector<double>& qp_time,
                            const std::vector<double>& qp_velocity,
                            const std::vector<double>& qp_acceleration,
                            const std::vector<double>& qp_jerk,
                            const double& dt)
    {
        std::ofstream writing_file;
        writing_file.open(filename, std::ios::out);
        writing_file << "qp_time" << "," << "qp_position" <<  "," << "qp_velocity" << "," << "qp_acceleration" << "," << "qp_jerk" << std::endl;

        double s = 0.0;
        for(int i=0; i<qp_time.size(); ++i)
        {
            writing_file << qp_time[i] << "," << s << "," << qp_velocity[i] << "," << qp_acceleration[i] << "," << qp_jerk[i] << std::endl;
            s = s + qp_velocity[i] * dt + 0.5 * qp_acceleration[i] * dt * dt;
        }

        writing_file.close();
    }

    void outputResultToFile(const std::string& filename,
                            const AStarOptimizer::AStarOutputInfo& output_info)
    {
        std::ofstream writing_file;
        writing_file.open(filename, std::ios::out);
        writing_file << "astar_time" << "," << "astar_position" << ","
                     << "astar_velocity" << "," << "astar_acceleration" << std::endl;

        for(int i=0; i<output_info.optimum_acceleration.size(); ++i)
            writing_file << output_info.optimum_time[i] << "," << output_info.optimum_position[i] << ","
                         << output_info.optimum_velocity[i] << "," << output_info.optimum_acceleration[i] << std::endl;

        writing_file.close();
    }
}
