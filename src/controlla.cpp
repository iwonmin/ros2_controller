#include "controlla.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

CmdPublisher::CmdPublisher() : Node("cmd_publisher") {
    // Publisher
    pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_marker = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    // Subscriber
    sub_octomap = this->create_subscription<OctomapMsg>("octomap_full", 10, std::bind(&CmdPublisher::octomap_callback, this, _1));

    rclcpp::QoS qos_laser(rclcpp::KeepLast(10));
    qos_laser.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", qos_laser, std::bind(&CmdPublisher::laser_callback, this, _1));
    sub_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10, std::bind(&CmdPublisher::goal_callback, this, _1));
    // TF listenerf
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    // Timer
    timer_tf = this->create_wall_timer(50ms, std::bind(&CmdPublisher::timer_tf_callback, this));
    timer_cmd = this->create_wall_timer(50ms, std::bind(&CmdPublisher::timer_cmd_callback, this));
    // timer_octomap_reset = this->create_wall_timer(1s, std::bind(&CmdPublisher::clear_obstacle, this));
    // Parameters
    this->declare_parameter("angular_kp", 1.73f);
    this->declare_parameter("angular_ki", 0.173f);
    this->declare_parameter("angular_kd", 0.41f);
    this->declare_parameter("linear_kp", 0.15f);
    this->declare_parameter("linear_ki", 0.10f);
    this->declare_parameter("linear_kd", 0.20f);
    angular.SetPID(this->get_parameter("angular_kp").as_double(), this->get_parameter("angular_ki").as_double(), this->get_parameter("angular_kd").as_double());
    linear.SetPID(this->get_parameter("linear_kp").as_double(), this->get_parameter("linear_ki").as_double(), this->get_parameter("linear_kd").as_double());
}

// A* Path Planning
std::vector<geometry_msgs::msg::PoseStamped> CmdPublisher::plan_path(float start_x, float start_y, float goal_x, float goal_y) {
    std::vector<geometry_msgs::msg::PoseStamped> path;
    std::priority_queue<AstarNode, std::vector<AstarNode>, Compare> open_list;
    std::set<std::pair<int, int>> closed_set;
    const int max_iterations = 10000;
    int iterations = 0;
    if(hypot(goal_x - start_x, goal_y - start_y)>search_radius) {
        float min_distance = std::numeric_limits<float>::max();
        float new_goal_x = start_x, new_goal_y = start_y;
        for (float angle = 0; angle < 2 * M_PI; angle += M_PI / 180) { 
            float nx = start_x + search_radius * cos(angle);
            float ny = start_y + search_radius * sin(angle);
            if (!map.is_free(nx, ny)) continue;
            float distance_to_goal = hypot(nx - goal_x, ny - goal_y);
            if (distance_to_goal < min_distance) {
                min_distance = distance_to_goal;
                new_goal_x = nx;
                new_goal_y = ny;
            }
        }
        // local_goal
        goal_x = new_goal_x;
        goal_y = new_goal_y;
    }
    closed_set.insert(to_grid(start_x, start_y, resolution));
    AstarNode start_node{start_x, start_y, 0, heuristic(start_x, start_y, goal_x, goal_y), nullptr};
    open_list.push(start_node);

    if (!map.is_free(start_x, start_y)) {
        RCLCPP_ERROR(this->get_logger(), "Start point is occupied!");
        return path;
    }

    if (!map.is_free(goal_x, goal_y)) {
        RCLCPP_ERROR(this->get_logger(), "Goal point is occupied!");
        return path;
    }
    while (!open_list.empty() && iterations < max_iterations) {
        AstarNode current = open_list.top(); 
        open_list.pop(); 
        iterations++;
        // RCLCPP_INFO_STREAM(this->get_logger(),"planning..");
        if (is_goal_path(current.x, current.y, goal_x, goal_y)) {
            return reconstruct_path(current);
        }


    closed_set.insert(to_grid(current.x, current.y, resolution));

    for (auto neighbor : get_neighbors(current)) {
        if (hypot(neighbor.x - start_x, neighbor.y - start_y) > search_radius) {
        continue;
        }
        if (closed_set.count(to_grid(neighbor.x, neighbor.y, resolution)) > 0) continue;

        neighbor.g = current.g + distance(current, neighbor);
        neighbor.h = heuristic(neighbor.x, neighbor.y, goal_x, goal_y);
        neighbor.parent = std::make_shared<AstarNode>(current);

        open_list.push(neighbor);
        }
    }
    if (iterations >= max_iterations) {
        RCLCPP_ERROR(this->get_logger(), "Exceeded maximum iterations, path not found!");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Path not found!");
    }
    
    return path;
}

std::pair<int, int> CmdPublisher::to_grid(float x, float y, float resolution) {
    return {static_cast<int>(x / resolution), static_cast<int>(y / resolution)};
}

float CmdPublisher::heuristic(float x1, float y1, float x2, float y2) {
    return hypot(x2 - x1, y2 - y1);
}

float CmdPublisher::distance(const AstarNode &a, const AstarNode &b) {
    return hypot(b.x - a.x, b.y - a.y);
}
bool CmdPublisher::is_goal(float x, float y, float goal_x, float goal_y) {
    return hypot(x - goal_x, y - goal_y) < 0.3; 
}
bool CmdPublisher::is_goal_path(float x, float y, float goal_x, float goal_y) {
    return hypot(x - goal_x, y - goal_y) < 0.3; 
}

std::vector<AstarNode> CmdPublisher::get_neighbors(const AstarNode &current) {

    std::vector<AstarNode> neighbors;

    std::vector<std::pair<float, float>> directions = { {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1} };

    for (const auto &dir : directions) {
        float nx = current.x + dir.first * resolution;
        float ny = current.y + dir.second * resolution;

        if (map.is_free(nx, ny)) {
        neighbors.emplace_back(AstarNode{nx, ny, 0, 0, nullptr});
        }
    }

    return neighbors;
}

std::vector<geometry_msgs::msg::PoseStamped> CmdPublisher::reconstruct_path(const AstarNode &goal_node) {
    std::vector<geometry_msgs::msg::PoseStamped> path;
    const AstarNode *current = &goal_node;

    while (current != nullptr) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = current->x;
        pose.pose.position.y = current->y;
        path.push_back(pose);

        current = current->parent.get();
    }

    std::reverse(path.begin(), path.end());
    return path;
}

void CmdPublisher::waypoint_visualize(const std::vector<geometry_msgs::msg::PoseStamped>& path) {
    visualization_msgs::msg::Marker marker;
    
    marker.header.frame_id = "map";  // rviz의 reference frame
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "path_visualizer";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;  // 경로를 라인으로 시각화
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.01;  // 라인의 두께
    
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    // 경로 점 추가
    for (const auto& pt : path) {
        geometry_msgs::msg::Point p;
        p.x = pt.pose.position.x;
        p.y = pt.pose.position.y;
        p.z = pt.pose.position.z;
        marker.points.push_back(p);
    }
    
    pub_marker->publish(marker);
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
    yaw = normalize_angle(yaw);
    position_updated = true;
}

void CmdPublisher::timer_cmd_callback() {
    if(not map.is_updated() or not position_updated) {
        RCLCPP_WARN(this->get_logger(), "map or tf not updated.");
        return;
    }
    // octomap::point3d search_point(x, y, 0);
    float obstacle_distance = minimum_distance;
    float obstacle_angle = normalize_angle(minimum_distance_angle);
    // octomap::point3d closest_obstacle(x + cos(obstacle_angle) * obstacle_distance, y + sin(obstacle_angle) * obstacle_distance, 0);
    // map.get_distance_and_closest_obstacle(search_point, distance, closest_obstacle);
    if (obstacle_distance < OBSTACLE_THRESHOLD || evasion) {
        evasion = true;
        RCLCPP_WARN(this->get_logger(), "Obstacle too close! Avoiding.");
        geometry_msgs::msg::Twist cmd_vel;
        if(fabs(minimum_distance_angle) < M_PI_4) {
            cmd_vel.linear.x = -0.15f;
        } else if(fabs(minimum_distance_angle) > 3 * M_PI_4) {
            cmd_vel.linear.x = +0.15f;
        }
        cmd_vel.angular.z = obstacle_angle > 0 ? 0.5 : -0.5;
        pub_cmd->publish(cmd_vel);
        if(obstacle_distance > 0.4f) {
            evasion = false;
            path = plan_path(x, y, goal.pose.position.x, goal.pose.position.y);
            waypoint_visualize(path);
        }
        minimum_distance = std::numeric_limits<float>::max();
        return;
    }
    float path_followed = hypot(x - start_point.x(), y - start_point.y());
    if(goal_state == GoalState::NOT_ACHIEVED && (path_followed >= 0.5f || goal_received)) {
        goal_received = false; 
        if(!deviation) path = plan_path(x, y, goal.pose.position.x, goal.pose.position.y);
        waypoint_visualize(path);
        start_point = octomap::point3d(x, y, z);
    }
    Controller();
}

// void CmdPublisher::clear_obstacle() {
//     octomap::point3d origin(x, y, 0);
//     std::vector<octomap::point3d> ray;
//     auto range = lidar_value;
//     for (int i = 0; i < 360; i++) {
//         if(std::isinf(range[i])) continue;
//         float angle = yaw + i * angle_increment;
//         octomap::point3d end(x + cos(angle) * range[i], y + sin(angle) * range[i], 0);
//         map.octree_ptr->computeRay(origin, end, ray);
//         for (const octomap::point3d point : ray) {
//             map.octree_ptr->deleteNode(point);
//         }
//     }
    

// }

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
}

void CmdPublisher::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    range_max = msg->range_max;
    range_min = msg->range_min;
    angle_increment = msg->angle_increment;
    for (int i = 0; i < msg->ranges.size(); i++) {
        lidar_value[i] = msg->ranges[i];
        if(lidar_value[i] < minimum_distance) {
            minimum_distance = lidar_value[i];
            minimum_distance_angle = i * angle_increment;
        }
    }
    // RCLCPP_INFO(this->get_logger(), "%.2f, %.2f", (180 * minimum_distance_angle / M_PI), minimum_distance);
    
}

void CmdPublisher::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_received = true;
    goal = *msg;
    goal_state = GoalState::NOT_ACHIEVED;
    RCLCPP_INFO(this->get_logger(), "Goal: %f, %f", goal.pose.position.x, goal.pose.position.y);
}

void CmdPublisher::Controller() {
    geometry_msgs::msg::Twist cmd_vel;
    switch (goal_state) {
        case GoalState::NOT_ACHIEVED:{
            if (path.empty()) return;
            geometry_msgs::msg::PoseStamped target = path.front(); 
            [[maybe_unused]] float distance_to_target = hypot(x - target.pose.position.x, y - target.pose.position.y);
            [[maybe_unused]] float distance_to_goal = hypot(x - goal.pose.position.x, y - goal.pose.position.y);
            if (distance_to_target < GOAL_THRESHOLD) {
                path.erase(path.begin());
                if (path.empty()) {
                    goal_state = GoalState::POSITION_ACHIEVED;
                    RCLCPP_INFO(this->get_logger(), "Goal position achieved!");
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    pub_cmd->publish(cmd_vel);
                    return;
                }
                prev_target = target;
                target = path.front();
            }
            float angle_to_target = atan2(target.pose.position.y - y, target.pose.position.x - x);
            if (std::abs(angle_to_target - yaw) > 0.4f || deviation) {
                deviation = true;
                RCLCPP_INFO(this->get_logger(), "Deviation Detected, Orienting..");
                // double m = (target.pose.position.y - prev_target.pose.position.y) / (target.pose.position.x - prev_target.pose.position.x);
                // double b = target.pose.position.y - m * target.pose.position.x;
                // double m_r = std::tan(yaw);
                // double b_r = y - m_r * x;
                // double x_i = (b_r - b) / (m - m_r);
                // double y_i = m * x_i + b;
                // double distance_to_intersection = hypot(x_i - x, y_i - y);
                // if(distance_to_intersection > 0.1f) {
                //     cmd_vel.angular.z = 0.f;
                //     cmd_vel.linear.x = std::clamp(linear.Output(distance_to_intersection),-max_linear, max_linear);
                //     pub_cmd->publish(cmd_vel);
                //     return;
                // }
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = std::clamp(angular.Output(angle_to_target, yaw, true), -max_angular, max_angular);
                // RCLCPP_INFO(this->get_logger(), "%.7f, %.7f", angle_to_target, yaw);
                pub_cmd->publish(cmd_vel);
                if(std::abs(angle_to_target - yaw) < 0.1f) deviation = false;
            } else {

                cmd_vel.linear.x = std::clamp(linear.Output(distance_to_target), -max_linear, max_linear);
                // cmd_vel.linear.x = 0.08f;
                // RCLCPP_INFO(this->get_logger(), "%.7f, %.7f", cmd_vel.linear.x, cmd_vel.angular.z);
                pub_cmd->publish(cmd_vel);
            }
            // angular.SetPID(1.2f, 0.0001f, 0.8f);
            // cmd_vel.angular.z = std::clamp(angular.Output(angle_to_target, yaw), -max_angular, max_angular);
            
            // linear.SetPID(1.0f, 0.01f, 0.5f);


            break;}
        case GoalState::POSITION_ACHIEVED:{
            double goal_yaw = atan2(2.0 * (goal.pose.orientation.z * goal.pose.orientation.w + goal.pose.orientation.x * goal.pose.orientation.y), 1.0 - 2.0 * (goal.pose.orientation.y * goal.pose.orientation.y + goal.pose.orientation.z * goal.pose.orientation.z));
            angular.SetPID(1.7f, 0.12f, 0.5f);
            cmd_vel.angular.z = std::clamp(angular.Output(goal_yaw, yaw, true), -max_angular, max_angular);
            pub_cmd->publish(cmd_vel);
            if (std::abs(angular.error) < 0.1f) {
                goal_state = GoalState::ACHIEVED;
                RCLCPP_INFO(this->get_logger(), "Goal orientation achieved!");
            }
            break;}
        case GoalState::ACHIEVED:{
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            pub_cmd->publish(cmd_vel);
            break;}
        }
}

float CmdPublisher::normalize_angle(float angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}
