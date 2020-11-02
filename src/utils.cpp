#include "utils.h"

namespace Utils
{
    void outputResultToFile(const std::string& filename,
                            const std::vector<double>& qp_time,
                            const std::vector<double>& qp_velocity,
                            const std::vector<double>& qp_acceleration,
                            const std::vector<double>& qp_jerk)
    {
        std::ofstream writing_file;
        writing_file.open(filename, std::ios::out);
        writing_file << "qp_time" << "," << "qp_velocity" << "," << "qp_acceleration" << "," << "qp_jerk" << std::endl;

        for(int i=0; i<qp_time.size(); ++i)
            writing_file << qp_time[i] << "," << qp_velocity[i] << "," << qp_acceleration[i] << "," << qp_jerk[i] << std::endl;

        writing_file.close();
    }

    void outputResultToFile(const std::string& filename,
                            const std::vector<double>& astar_time,
                            const std::vector<double>& astar_velocity,
                            const std::vector<double>& astar_acceleration)
    {
        std::ofstream writing_file;
        writing_file.open(filename, std::ios::out);
        writing_file << "astar_time" << "," << "astar_velocity" << "," << "astar_acceleration" << std::endl;

        for(int i=0; i<astar_time.size(); ++i)
            writing_file << astar_time[i] << "," << astar_velocity[i] << "," << astar_acceleration[i] << std::endl;

        writing_file.close();
    }
}
