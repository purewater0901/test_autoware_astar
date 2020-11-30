#include "hastar_optimizer.h"

double HAStarOptimizer::calculateHeuristicCost(const double& s, const double& goal_s)
{
    return 10*std::pow(s-goal_s, 2);
}

double HAStarOptimizer::calculateActualCost(const double& v, const double& reference_v,
                                           const double& weight_v,
                                           const double& current_a, const double& next_a, const double& dt,
                                           const double& weight_a, const double& offset)
{
    return offset + weight_v * std::pow((v - reference_v), 2) + weight_a*std::pow((current_a-next_a)/dt, 2);;
}

double HAStarOptimizer::calculateMaximumVelocity(const int& index,
                                                const std::vector<double>& vmax_vec)
{
    double vmax;
    if(index>=vmax_vec.size())
        vmax = vmax_vec.back();
    else
        vmax = vmax_vec[index];

    return vmax;
}

void HAStarOptimizer::createNewNode(std::set<HAStarNode*>& open_node_list,
                                   std::set<HAStarNode*>& closed_node_list,
                                   HAStarNode* current_node,
                                   const double& weight_v, const double& weight_a, const double& goal_s,
                                   const double& next_s, const double& next_v, const double& next_t,
                                   const double& next_max_v, const double& a_command,
                                   const int& next_s_id, const int& next_v_id, const int& next_t_id)
{
    // Calculate New Node costs
    double current_a = current_node->getAcceleration();
    double dt = next_t - current_node->getTime();
    double ref_v = std::max(0.0, next_max_v-0.1);
    double current_cost = current_node->getActualCost();
    double actual_cost = calculateActualCost(next_v, ref_v, weight_v, current_a, a_command, dt, weight_a, current_cost) + 1000*std::pow(std::fmax(0, next_v-next_max_v), 2);
    double heuristic_cost = calculateHeuristicCost(next_s, goal_s);


    // Check if new node is already in the closed node list
    HAStarNode* closed_node =
            NodeUtils::findNodeOnList(closed_node_list, next_s_id, next_t_id, next_v_id);
    if (closed_node != nullptr) return; // New node is already in the closed node list
    /*
    if (closed_node != nullptr && actual_cost<closed_node->getActualCost())
    {
      //update closed node list and erase it from the closed nose list
      closed_node->setPosition(next_s);
      closed_node->setTime(next_t);
      closed_node->setVelocity(next_v);
      closed_node->setAcceleration(a_command);
      closed_node->setActualCost(actual_cost);
      closed_node->setHeuristicCost(heuristic_cost);
      closed_node->setParentNode(current_node);
      open_node_list.insert(closed_node);
      closed_node_list.erase(closed_node_list.find(closed_node));
      return;
    }
    else if (closed_node != nullptr) // New node is already in the closed node list
      return;
      */

    // Check if the new node is in the open node list
    HAStarNode* successor =
            NodeUtils::findNodeOnList(open_node_list, next_s_id, next_t_id, next_v_id);

    if (successor == nullptr) {
        //We don't have this new node in the open node list
        successor =
                new HAStarNode(next_s, next_t, next_v, a_command, next_s_id, next_t_id, next_v_id,
                               actual_cost, heuristic_cost, goal_s, current_node);
        open_node_list.insert(successor);
    } else if (actual_cost < successor->getActualCost()) {
        /*
           We already has this node in the open node list, and the node cost is lower than original,
           so we will update the cost
         */
        successor->setPosition(next_s);
        successor->setTime(next_t);
        successor->setVelocity(next_v);
        successor->setAcceleration(a_command);
        successor->setActualCost(actual_cost);
        successor->setHeuristicCost(heuristic_cost);
        successor->setParentNode(current_node);
    }
}

bool HAStarOptimizer::calculateByFixDistance(const double& initial_vel,
                                            const double& initial_acc,
                                            const unsigned int& N,
                                            const double& ds,
                                            const double& dt,
                                            const double& dv,
                                            const double& goal_s,
                                            const double& offset,
                                            const double& tol,
                                            const double& t_max,
                                            const double& weight_v,
                                            const double& weight_a,
                                            const std::vector<double>& output_trajectory_index,
                                            const std::vector<double>& interpolated_max_velocity,
                                            const std::vector<double>& da_list,
                                            std::vector<double>& optimum_position,
                                            std::vector<double>& optimum_velocity,
                                            std::vector<double>& optimum_acceleration,
                                            std::vector<double>& optimum_time)
{
    // node list
    std::set<HAStarNode*> open_node_list;
    std::set<HAStarNode*> closed_node_list;

    // initial node
    double s0 = 0.0;
    double t0 = 0.0;
    double v0 = initial_vel;
    double a0 = initial_acc;
    int s_d = static_cast<int>(s0/ds);
    int t_d = static_cast<int>(t0/dt);
    int v_d = static_cast<int>(v0/dv);
    double initial_heuristic_cost = this->calculateHeuristicCost(s0, goal_s);
    HAStarNode initial_node(s0, t0, v0, a0, s_d, t_d, v_d, initial_heuristic_cost, goal_s);
    open_node_list.insert(&initial_node);


    // solve the problem
    optimum_position.reserve(output_trajectory_index.size());
    optimum_velocity.reserve(output_trajectory_index.size());
    optimum_acceleration.reserve(output_trajectory_index.size());
    optimum_time.reserve(output_trajectory_index.size());
    while(true)
    {
        if(open_node_list.empty())
        {
            std::cerr << "[MotionVelocityOptimizer] A Star Node list failed to find a solution." << std::endl;
            exit(1);
            return false;
        }

        // Choose the least cost node
        HAStarNode* current_node = *open_node_list.begin();
        for(HAStarNode* node : open_node_list)
            if(node->getScore() <= current_node->getScore())
                current_node = node;

        //check current node reaches the goal
        if(current_node->isGoal(goal_s))
        {
            std::cout << "[VelocityOptimizer]: AStar Velocity Optimizer find a solution" << std::endl;
            while(current_node != nullptr)
            {
                optimum_position.push_back(current_node->getPosition());
                optimum_velocity.push_back(current_node->getVelocity());
                optimum_acceleration.push_back(current_node->getAcceleration());
                optimum_time.push_back(current_node->getTime());
                current_node = current_node->getParentNode();
            }
            std::reverse(optimum_position.begin(), optimum_position.end());
            std::reverse(optimum_velocity.begin(), optimum_velocity.end());
            std::reverse(optimum_acceleration.begin(), optimum_acceleration.end());
            std::reverse(optimum_time.begin(), optimum_time.end());


            for(int i=0; i<optimum_position.size(); ++i){
                double v = optimum_velocity.at(i);
                printf("s[%d] %f  v[%d] %f  a[%d] %f \n", i, optimum_position[i], i, v, i, optimum_acceleration[i]);
            }

            return true;
        }

        // Insert current node into closed node list and erase current node from open node list
        closed_node_list.insert(current_node);
        open_node_list.erase(open_node_list.find(current_node));


        // Get Current Position Information
        double current_s = current_node->getPosition();
        double current_t = current_node->getTime();
        double current_v = current_node->getVelocity();


        // Update neighbors
        for(long unsigned int i=0; i<da_list.size(); ++i)
        {
            // Acceleration
            double a_command = da_list[i];


            //Next node information
            double next_a;
            double next_s;
            double next_v;
            double next_t;
            double next_max_v;
            int next_s_id;
            int next_v_id;
            int next_t_id;

            // Give a special process when the vehicle stops and have non-positive acceleration
            if(current_v < 1e-6 && a_command <= 0.0){

                std::cout << "Current s: " << current_s << std::endl;
                std::cout << "Current t: " << current_t << std::endl;
                std::cout << "Current v: " << current_v << std::endl;
                std::cout << "------------" << std::endl;
                next_a = 0.0; // Assume acceleration is zero
                next_t = current_t + dt; // Do calculation while fixing dt
                next_s = current_s;
                next_v = current_v;
                next_t_id = current_node->getTimeIndex() + 1;
                next_s_id = current_node->getPositionIndex();
                next_v_id = current_node->getVelocityIndex();
                next_max_v = calculateMaximumVelocity(next_s_id, interpolated_max_velocity) + offset;
                int next_max_v_id = static_cast<int>(std::round(next_max_v/dv));
                if(next_v_id>next_max_v_id) continue;

            }else{
                // update position
                next_s = current_s + ds;
                next_s_id = current_node->getPositionIndex() + 1;

                // update velocity
                next_v = std::sqrt(std::max(0.0, current_v*current_v+2*a_command*ds));
                next_v_id = static_cast<int>(std::round(next_v / dv));
                next_max_v = calculateMaximumVelocity(next_s_id, interpolated_max_velocity) + offset;
                int next_max_v_id = static_cast<int>(std::round((next_max_v)/dv));
                std::cout << "Current s: " << current_s << std::endl;
                std::cout << "Current t: " << current_t << std::endl;
                std::cout << "Current v: " << current_v << std::endl;
                std::cout << "acceleration: " << a_command << std::endl;
                std::cout << "next_s_id: " << next_s_id<< std::endl;
                std::cout << "next_v: " << next_v << std::endl;
                std::cout << "next_max_v: " << next_max_v << std::endl;
                std::cout << "next_v_id: " << next_v_id << std::endl;
                std::cout << "next_max_v_id: " << next_max_v_id << std::endl;
                std::cout << "---------------------------" << std::endl;
                if(next_v_id>next_max_v_id)
                {
                    std::cout << "Out" << std::endl;
                    continue;
                }

                // update acceleration
                next_a = (next_v*next_v - current_v*current_v)/(2*ds);

                //  update time
                double t_increase = (std::fabs(next_a) <= tol) ? ds / current_v
                                                               : std::fabs(next_v - current_v) / std::fabs(next_a);
                next_t = current_t + t_increase;
                next_t_id =
                        current_node->getTimeIndex() + static_cast<int>(std::round(t_increase / dt));
            }

            if (next_t > t_max) continue;
            /*
            std::cout << "next_s: " << next_s << std::endl;
            std::cout << "current_v: " << current_node->getVelocity()<< std::endl;
            std::cout << "next_v: " << next_v << std::endl;
            std::cout << "next_t: " << next_t << std::endl;
            std::cout << "next_a: " << next_a << std::endl;
            std::cout << "-----------" << std::endl;
             */


            createNewNode(open_node_list, closed_node_list, current_node, weight_v, weight_a, goal_s,
                          next_s, next_v, next_t, next_max_v, next_a,
                          next_s_id, next_v_id, next_t_id);
        }
    }
}

bool HAStarOptimizer::solve(const double initial_vel,
                           const double initial_acc,
                           const int N,
                           const double goal_s,
                           const std::vector<double>& output_trajectory_index,
                           const std::vector<double>& interpolated_max_velocity,
                           std::vector<double>& optimum_position,
                           std::vector<double>& optimum_velocity,
                           std::vector<double>& optimum_acceleration,
                           std::vector<double>& optimum_time)
{
    // parameters
    double tol = 1e-8;
    double t_max = 20.0;
    double weight_v = 1.0;
    double weight_a = 100.0;
    double ds = 1.0;
    double dt = 0.5;
    //double dv = param_.max_accel * dt;
    double dv = 0.01;
    std::vector<double> da_list = {1.0, 0.0, -1.0};
    double offset = 3.0;

    bool is_success = calculateByFixDistance(initial_vel, initial_acc, N, ds, dt, dv, goal_s, offset, tol,
                                             t_max, weight_v, weight_a, output_trajectory_index,
                                             interpolated_max_velocity, da_list,
                                             optimum_position, optimum_velocity, optimum_acceleration, optimum_time);

    return is_success;
}
