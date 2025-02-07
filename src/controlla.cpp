#include "controlla.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

CmdPublisher::CmdPublisher() : Node("cmd_publisher") {
    // Publisher
    pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // Subscriber
    sub_octomap = this->create_subscription<OctomapMsg>("octomap_full", 10, std::bind(&CmdPublisher::octomap_callback, this, _1));
    sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&CmdPublisher::laser_callback, this, _1));
    sub_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10, std::bind(&CmdPublisher::goal_callback, this, _1));
    // TF listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    // Timer
    timer_tf = this->create_wall_timer(50ms, std::bind(&CmdPublisher::timer_tf_callback, this));
    timer_cmd = this->create_wall_timer(100ms, std::bind(&CmdPublisher::timer_cmd_callback, this));
}

void CmdPublisher::timer_tf_callback() {
    geometry_msgs::msg::TransformStamped t;
    try {
        t = tf_buffer->lookupTransform("map", "base_scan", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Could not transform map to base_scan");
        return;
    }

    x = t.transform.translation.x;
    y = t.transform.translation.y;
    z = t.transform.translation.z;
    
    pose_x = t.transform.rotation.x;
    pose_y = t.transform.rotation.y;
    pose_z = t.transform.rotation.z;
    pose_w = t.transform.rotation.w;
    yaw = atan2(2.0 * (pose_z * pose_w + pose_x * pose_y), 1.0 - 2.0 * (pose_y * pose_y + pose_z * pose_z));
    position_updated = true;
}

void CmdPublisher::timer_cmd_callback() {
    if(not map.is_updated() or not position_updated) {
        RCLCPP_WARN(this->get_logger(), "map or tf not updated.");
        return;
    }
    octomap::point3d search_point(x,y,z);
    if (!goal_received) {
        RCLCPP_WARN(this->get_logger(), "Where should I go?");
        return;
    }
    if (goal_state == GoalState::NOT_ACHIEVED) {
        a_star_planner(x, y, goal.pose.position.x, goal.pose.position.y, grid);
        dwa_algorithm(path_index);
    } else if(goal_state == GoalState::POSITION_ACHIEVED) goal_orientation_achievement();
}

void CmdPublisher::octomap_callback(const OctomapMsg& octomap_msg) {
    octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(octomap_msg);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
    if (!octree) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert Octomap message to OcTree");
        return;
    }
    octomap::point3d world_min(-5, -5, 0);
    octomap::point3d world_max(5, 5, 2);  
    map.update(octomap_msg, world_min, world_max);

    std::vector<std::vector<int>> grid = create_grid_map_from_octomap(*octree);
    // Clean up the dynamically allocated tree
    delete octree;
}

std::vector<std::vector<int>> CmdPublisher::create_grid_map_from_octomap(const octomap::OcTree& octree) {
    std::vector<std::vector<int>> grid(grid_height, std::vector<int>(grid_width, 0));

    for (octomap::OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it) {
        if (octree.isNodeOccupied(*it)) {
            float x = it.getX();
            float y = it.getY();
            float z = it.getZ();

            // Project to 2D plane (assuming z = 0 is the ground plane)
            int grid_x = static_cast<int>((x + grid_width * grid_resolution / 2) / grid_resolution);
            int grid_y = static_cast<int>((y + grid_height * grid_resolution / 2) / grid_resolution);

            if (grid_x >= 0 && grid_x < grid_width && grid_y >= 0 && grid_y < grid_height) {
                grid[grid_y][grid_x] = 1; // Mark as occupied
            }
        }
    }

    return grid;
}

void CmdPublisher::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float angle_max = msg->angle_max;
    float angle_min = msg->angle_min;
    float angle_increment = msg->angle_increment;
    uint8_t lidar_range_size = msg->ranges.size();
    for(uint8_t i=0;i++;i<lidar_range_size) {
        lidar_value[i] = msg->ranges[i];
    }
}

void CmdPublisher::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_received = true;
    goal = *msg;
    goal_state = GoalState::NOT_ACHIEVED;
    RCLCPP_INFO(this->get_logger(), "Goal: %f, %f, %f, %f", goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.z, goal.pose.orientation.w);
    
    // Call A* path planner
    std::vector<std::array<float, 3>> path = a_star_planner(x, y, goal.pose.position.x, goal.pose.position.y, grid);

    // Divide the path into 10 waypoints
    divide_path_into_waypoints(path, 10);
    current_waypoint_index = 0;
}

void CmdPublisher::goal_orientation_achievement() {
    //잘됨
    double goal_yaw = atan2(2.0 * (goal.pose.orientation.z * goal.pose.orientation.w + goal.pose.orientation.x * goal.pose.orientation.y), 1.0 - 2.0 * (goal.pose.orientation.y * goal.pose.orientation.y + goal.pose.orientation.z * goal.pose.orientation.z));
    double error = goal_yaw - yaw;
    error = normalize_angle(error);
    integral += error;
    double derivative = error - prev_error;
    double output = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;
    // Apply PID output to angular velocity
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.angular.z = std::clamp(output, -max_angular, max_angular);
    pub_cmd->publish(cmd_vel);

    if (std::abs(error) < 0.1) {
        goal_state = GoalState::ACHIEVED;
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        pub_cmd->publish(cmd_vel);
        RCLCPP_INFO(this->get_logger(), "Goal orientation achieved!");
        integral = 0.0;
        prev_error = 0.0;

    }
}
std::vector<std::array<float, 3>> CmdPublisher::a_star_planner(float start_x, float start_y, float goal_x, float goal_y, const std::vector<std::vector<int>>& grid) {
    std::priority_queue<astarNode, std::vector<astarNode>, std::greater<astarNode>> open_list;
    std::unordered_map<astarNode, astarNode*, astarNodeHash, astarNodeEqual> all_nodes;

    astarNode* start_node = new astarNode(start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y));
    open_list.push(*start_node);
    all_nodes[*start_node] = start_node;

    std::vector<std::array<float, 3>> path;

    while (!open_list.empty()) {
        astarNode current_node = open_list.top();
        open_list.pop();

        // Check if the goal is reached
        if (std::abs(current_node.x - goal_x) < 0.1 && std::abs(current_node.y - goal_y) < 0.1) {
            astarNode* node = &current_node;
            while (node != nullptr) {
                path.push_back({node->x, node->y, 0.0}); // Add yaw if needed
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            break;
        }

        // Generate neighbors
        std::vector<std::pair<float, float>> neighbors = {
            {current_node.x + 1, current_node.y},
            {current_node.x - 1, current_node.y},
            {current_node.x, current_node.y + 1},
            {current_node.x, current_node.y - 1}
        };

        for (const auto& neighbor : neighbors) {
            float neighbor_x = neighbor.first;
            float neighbor_y = neighbor.second;

            // Check if the neighbor is within the grid and not an obstacle
            if (neighbor_x >= 0 && neighbor_x < grid.size() && neighbor_y >= 0 && neighbor_y < grid[0].size() && grid[neighbor_x][neighbor_y] == 0) {
                float g_cost = current_node.g_cost + 1.0;
                float h_cost = heuristic(neighbor_x, neighbor_y, goal_x, goal_y);
                astarNode* neighbor_node = new astarNode(neighbor_x, neighbor_y, g_cost, h_cost, all_nodes[current_node]);

                if (all_nodes.find(*neighbor_node) == all_nodes.end() || g_cost < all_nodes[*neighbor_node]->g_cost) {
                    open_list.push(*neighbor_node);
                    all_nodes[*neighbor_node] = neighbor_node;
                }
            }
        }
    }

    // Clean up allocated nodes
    for (auto& pair : all_nodes) {
        delete pair.second;
    }

    return path;
}

void CmdPublisher::divide_path_into_waypoints(const std::vector<std::array<float, 3>>& path, int num_waypoints) {
    std::vector<std::array<float, 3>> waypoints;
    int path_size = path.size();
    float step = static_cast<float>(path_size - 1) / (num_waypoints - 1);
    for (int i = 0; i < num_waypoints; ++i) {
        int index = static_cast<int>(i * step);
        waypoints[index] = path[index];
    }

}

void CmdPublisher::dwa_algorithm(int path_index) {
    if (!goal_received) {
        RCLCPP_WARN(this->get_logger(), "Where should I go?");
        return;
    }

    if (hypot(goal.pose.position.x - x, goal.pose.position.y - y) < GOAL_THRESHOLD) {
        goal_state = GoalState::POSITION_ACHIEVED;
        RCLCPP_INFO(this->get_logger(), "Goal position achieved!");
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.z = 0.0;
        pub_cmd->publish(cmd_vel);
        return;
    }

    // 장애물 거리 확인
    float nearest_obstacle_distance;
    octomap::point3d closest_obstacle;
    map.get_distance_and_closest_obstacle({x, y, z}, nearest_obstacle_distance, closest_obstacle);
    /*
    float obs_dx = closest_obstacle.x() - x;
    float obs_dy = closest_obstacle.y() - y;
    float obstacle_angle = std::atan2(obs_dy, obs_dx);
    float angle_diff = normalize_angle(obstacle_angle - yaw);  // 현재 방향과 비교
    
    if (nearest_obstacle_distance < OBSTACLE_THRESHOLD && std::abs(angle_diff) < M_PI/3) {  
        // 장애물이 가까울 경우 반대 방향으로 회피
        RCLCPP_WARN(this->get_logger(), "Obstacle too close! Avoiding.");
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.04f;
        cmd_vel.angular.z = (angle_diff > 0) ? -max_angular : max_angular;  // 반대 방향으로 회전
        pub_cmd->publish(cmd_vel);
        return;
    }
    */
    // DWA 속도 샘플링
    std::vector<std::pair<float, float>> velocity_samples;
    for (float v = 0.0; v <= max_linear; v += 0.05) {     
        for (float w = -max_angular; w <= max_angular; w += 0.1) { 
            velocity_samples.push_back({v, w});
        }
    }

    float best_v = 0.0, best_w = 0.0;
    float best_cost = std::numeric_limits<float>::max();

    for (auto [v, w] : velocity_samples) {
        float heading_cost = goal_distance_score(v, w);
        float clearance_cost = obstacle_distance_score(nearest_obstacle_distance);
        float velocity_cost = (max_linear - v) * 0.5; // 너무 빠른 속도 억제
        float d_line_cost = deviation_from_line_cost(v, w);

        float total_cost = heading_cost * alpha 
                         + clearance_cost * beta 
                         + velocity_cost * gamma 
                         + d_line_cost * delta;

        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_v = v;
            best_w = w;
        }
    }
    /*
    // 디버깅용 로그 출력
    RCLCPP_INFO(this->get_logger(), 
                "Heading Cost: %f, Clearance Cost: %f, Velocity Cost: %f, D-Line Cost: %f", 
                alpha * goal_distance_score(best_v, best_w), 
                beta * obstacle_distance_score(nearest_obstacle_distance), 
                gamma * (max_linear - best_v) * 0.5,
                delta * deviation_from_line_cost(best_v, best_w));
    */
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = best_v;
    cmd_vel.angular.z = best_w;

    pub_cmd->publish(cmd_vel);
}

float CmdPublisher::goal_distance_score(float v, float w) {
    float dx = goal.pose.position.x - x;
    float dy = goal.pose.position.y - y;
    float target_angle = std::atan2(dy, dx);
    float heading_cost = std::abs(target_angle - w);
    return heading_cost;
}

float CmdPublisher::obstacle_distance_score(float nearest_obstacle_distance) {
    if (nearest_obstacle_distance < 0.3) return 100.0;  // 너무 큰 값이 아니라 적당한 패널티 부여
    return 1.0 / (nearest_obstacle_distance + 0.01);  // 0으로 나누는 오류 방지
}

float CmdPublisher::deviation_from_line_cost(float v, float w) {
    // 목표점과 현재 위치
    float goal_x = goal.pose.position.x;
    float goal_y = goal.pose.position.y;

    // 목표 방향 계산
    float target_angle = std::atan2(goal_y - y, goal_x - x);

    // 현재 이동 방향 예측 (오일러 적분 방식)
    float predicted_theta = yaw + w * 0.1;  // 0.1초 뒤 예측
    float predicted_x = x + v * cos(predicted_theta) * 0.1;
    float predicted_y = y + v * sin(predicted_theta) * 0.1;

    // 예측된 이동 경로가 직선 경로에서 얼마나 벗어나는지 계산
    float deviation = std::abs(std::atan2(predicted_y - y, predicted_x - x) - target_angle);

    return deviation;
}

float CmdPublisher::normalize_angle(float angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}