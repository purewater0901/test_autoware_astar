#include <iostream>
#include <vector>

#include "astar_optimizer.h"

int main() {

    //int N = 59;
    //double initial_velocity = 7.94965;
    //double initial_acceleration = -1.94025;
    int N = 55;
    double initial_velocity = 8.13652;
    double initial_acceleration = -1.99298;

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
    optimizer.solve(initial_velocity, initial_acceleration, N, s_goal, s_longitudinal, v_longitudinal);



    return 0;
}
