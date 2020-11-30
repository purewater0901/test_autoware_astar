#include "astar_optimizer.h"

AStarOptimizer::AStarOptimizer(const AStarOptimizerParam& param) : param_(param)
{}

double AStarOptimizer::calculateHeuristicCost(const double& s, const double& goal_s)
{
    return param_.weight_s * std::pow(s - goal_s, 2); //Position Cost for Heuristic Cost
}

double AStarOptimizer::calculateActualCost(const double& v, const double& reference_v,
                                           const double& current_a, const double& next_a, const double& dt,
                                           const double& offset)
{
    return offset + param_.weight_v * std::pow((v - reference_v), 2)  // Velocity Follow cost
           + param_.weight_jerk * std::pow((current_a-next_a)/dt, 2) // Jerk Cost
           + param_.weight_over_v * std::pow(std::fmax(0, v - reference_v), 2); // Maximum Velocity Constraint Soft Cost
}

double AStarOptimizer::calculateMaximumVelocity(const int& index,
                                                const std::vector<double>& vmax_vec)
{
    double vmax;
    if(index>=vmax_vec.size())
        vmax = vmax_vec.back();
    else
        vmax = vmax_vec[index];

    return vmax;
}

bool AStarOptimizer::solve(
        const double initial_vel, const double initial_acc, const int closest,
        const std::vector<double>& vmax,
        const std::vector<double>& input_trajectory_index,
        AStarOutputInfo& output_info)
{
    // Direction (******** Caution: We need to fix the s_direction to 1 *******)
    std::vector<std::pair<int, int>> directions = {{1, -1}, {1, 0}, {1, 1}};

    // Goal Position for this calculation
    const double goal_s = input_trajectory_index.back();

    // Calculate Interpolated Point Max Velocity
    std::vector<double> output_trajectory_index;
    output_trajectory_index.push_back(0.0); // initial position
    while(true)
    {
        double next_index = output_trajectory_index.back()+param_.ds;
        if(next_index>=goal_s)
        {
            output_trajectory_index.push_back(goal_s);
            break;
        }
        output_trajectory_index.push_back(next_index);
    }
    //LinearInterpolate::interpolate(input_trajectory_index, vmax, output_trajectory_index, interpolated_max_velocity);

    // Use Distance Fixed AStar to calculate optimum velocity and acceleration
    bool is_success = calculateByFixDistance(initial_vel, initial_acc, goal_s,
                                             output_trajectory_index,
                                             vmax,
                                             directions, output_info);

    return is_success;
}

void AStarOptimizer::createNewNode(std::set<AStarNode*>& open_node_list,
                                   std::set<AStarNode*>& closed_node_list,
                                   AStarNode* current_node,
                                   const double& goal_s,
                                   const NodeInformation& node_info)
{
    // Calculate New Node costs
    double current_a = current_node->getAcceleration();
    double dt = node_info.t - current_node->getTime();
    double current_cost = current_node->getActualCost();
    double actual_cost = calculateActualCost(node_info.v, node_info.max_v, current_a, node_info.a, dt, current_cost);
    double heuristic_cost = calculateHeuristicCost(node_info.s, goal_s);

    // Check if new node is already in the closed node list
    AStarNode* closed_node =
            NodeUtils::findNodeOnList(closed_node_list, node_info.s_id, node_info.t_id, node_info.v_id);
    if (closed_node != nullptr) return; // New node is already in the closed node list

    // Check if the new node is in the open node list
    AStarNode* successor =
            NodeUtils::findNodeOnList(open_node_list, node_info.s_id, node_info.t_id, node_info.v_id);

    if (successor == nullptr) {
        //We don't have this new node in the open node list
        successor = new AStarNode(node_info, actual_cost, heuristic_cost, goal_s, current_node);
        open_node_list.insert(successor);
    } else if (actual_cost < successor->getActualCost()) {
        /*
           We already has this node in the open node list, and the node cost is lower than original,
           so we will update the cost
         */
        successor->setPosition(node_info.s);
        successor->setTime(node_info.t);
        successor->setVelocity(node_info.v);
        successor->setAcceleration(node_info.a);
        successor->setActualCost(actual_cost);
        successor->setHeuristicCost(heuristic_cost);
        successor->setParentNode(current_node);
    }
}

bool AStarOptimizer::calculateByFixDistance(const double& initial_vel,
                                            const double& initial_acc,
                                            const double& goal_s,
                                            const std::vector<double>& output_trajectory_index,
                                            const std::vector<double>& max_velocity,
                                            const std::vector<std::pair<int, int>>& directions,
                                            AStarOutputInfo& output_info)
{
    // node list
    std::set<AStarNode*> open_node_list;
    std::set<AStarNode*> closed_node_list;

    // initial node
    NodeInformation init_node_info{};
    init_node_info.s = 0.0;
    init_node_info.t = 0.0;
    init_node_info.v = initial_vel;
    init_node_info.a = initial_acc;
    init_node_info.s_id = static_cast<int>(init_node_info.s/param_.ds);
    init_node_info.t_id = static_cast<int>(init_node_info.t/param_.dt);
    init_node_info.v_id = static_cast<int>(init_node_info.v/param_.dv);
    double initial_heuristic_cost = calculateHeuristicCost(init_node_info.s, goal_s);
    AStarNode initial_node(init_node_info, initial_heuristic_cost, goal_s);
    open_node_list.insert(&initial_node);


    // solve the problem
    output_info.reserve(output_trajectory_index.size());
    while(true)
    {
        if(open_node_list.empty())
        {
            std::cerr << "[MotionVelocityOptimizer] A Star Node list failed to find a solution." << std::endl;
            exit(1);
            return false;
        }

        // Choose the least cost node
        AStarNode* current_node = *open_node_list.begin();
        for(AStarNode* node : open_node_list)
            if(node->getScore() <= current_node->getScore())
                current_node = node;

        //check current node reaches the goal
        int goal_index = static_cast<int>(goal_s/param_.ds);
        if(goal_index <= current_node->getPositionIndex())
        {
            std::cout << "[VelocityOptimizer]: AStar Velocity Optimizer find a solution" << std::endl;
            while(current_node != nullptr)
            {
                output_info.push_back(current_node);
                current_node = current_node->getParentNode();
            }
            output_info.reverse();
            return true;
        }

        // Insert current node into closed node list and erase current node from open node list
        closed_node_list.insert(current_node);
        open_node_list.erase(open_node_list.find(current_node));

        // Get Current Position Information
        double current_s = current_node->getPosition();
        double current_t = current_node->getTime();
        double current_v = current_node->getVelocity();
        int current_s_id = current_node->getPositionIndex();
        int current_t_id = current_node->getTimeIndex();
        int current_v_id = current_node->getVelocityIndex();

        for(unsigned int i=0; i<directions.size(); ++i)
        {
            //Next node information
            NodeInformation next_node_info{};

            // Pick up direction
            int s_direction = directions[i].first;
            int v_direction = directions[i].second;
            assert(s_direction==1);

            if(current_v_id==0 && v_direction == 0)
            {
                // Update Position
                next_node_info.s_id = current_s_id;
                next_node_info.s    = current_s;

                // Update Velocity
                next_node_info.v_id = current_v_id;
                next_node_info.v = current_v;
                next_node_info.max_v = calculateMaximumVelocity(next_node_info.s_id, max_velocity);

                //Check Velocity Violation
                if(next_node_info.v_id > static_cast<int>(next_node_info.max_v/param_.ds))
                    continue;

                // Update acceleration
                next_node_info.a = 0.0;

                // Update time
                next_node_info.t    = current_t + param_.dt;
                next_node_info.t_id = current_t_id + 1;
            }
            else
            {
                // Update Position
                next_node_info.s_id = current_s_id + s_direction;
                next_node_info.s    = current_s + s_direction * param_.ds;

                // Update Velocity
                next_node_info.v_id = current_v_id + v_direction;
                next_node_info.v = std::max(0.0, current_v + v_direction * param_.dv);
                next_node_info.max_v = calculateMaximumVelocity(next_node_info.s_id, max_velocity);

                //Check Velocity Violation
                if(next_node_info.v_id > static_cast<int>(next_node_info.max_v/param_.dv) || next_node_info.v_id < 0)
                    continue;

                // Update acceleration
                next_node_info.a = (next_node_info.v * next_node_info.v- current_v*current_v)/(2*param_.ds);

                double t_increase;
                if(v_direction==0)
                    t_increase = param_.ds/current_v;
                else
                    t_increase = std::fabs((next_node_info.v - current_v)/next_node_info.a);

                next_node_info.t    = current_t + t_increase;
                next_node_info.t_id = current_t_id + static_cast<int>(t_increase/param_.dt);
            }

            if (next_node_info.t > param_.max_time) continue;

            createNewNode(open_node_list, closed_node_list, current_node, goal_s, next_node_info);
        }

    }
}