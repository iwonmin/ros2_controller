#include "controlla.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

CmdPublisher::CmdPublisher() : Node("cmd_publisher") {
    // Publisher
    pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // Subscriber
    sub_octomap = this->create_subscription<OctomapMsg>("octomap_full", 10, std::bind(&CmdPublisher::octomap_callback, this, _1));
    sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&CmdPublisher::laser_callback, this, _1));
    // TF listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    // Timer
    timer_tf = this->create_wall_timer(50ms, std::bind(&CmdPublisher::timer_tf_callback, this));
    timer_cmd = this->create_wall_timer(500ms, std::bind(&CmdPublisher::timer_cmd_callback, this));
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

    position_updated = true;
}

void CmdPublisher::timer_cmd_callback() {
    if(not map.is_updated() or not position_updated) {
        RCLCPP_WARN(this->get_logger(), "map or tf not updated.");
        return;
    }

    octomap::point3d search_point(x,y,z);
    octomap::point3d closest_obstacle;
    float distance;
    //Path & Control
    /* 
    map.get_distance_and_closest_obstacle(search_point, distance, closest_obstacle);
    RCLCPP_INFO_STREAM(this->get_logger(), "Distance to obstacle: " << std::to_string(distance) << ", Closest obsatcle: " << closest_obstacle);
    */
    geometry_msgs::msg::Twist cmd_vel;
    pub_cmd->publish(cmd_vel);

}

void CmdPublisher::octomap_callback(const OctomapMsg& octomap_msg) {
    octomap::point3d world_min(-5, -5, 0);
    octomap::point3d world_max(5, 5, 2);  
    map.update(octomap_msg, world_min, world_max);
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

void CmdPublisher::velocity_by_dwa() {
        if (nearest_obstacle_distance_ < OBSTACLE_THRESHOLD) {  // 장애물 너무 가까움 -> 정지
            RCLCPP_WARN(this->get_logger(), "Obstacle too close! Stopping.");
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.f;
            stop_cmd.angular.z = 0.f;
            
            pub_cmd->publish(stop_cmd);
            return;
        }

        // 🏁 DWA 기반 속도 탐색
        std::vector<std::pair<float, float>> velocity_samples;
        for (float v = 0.0; v <= max_linear; v += 0.05) {     // 선속도 샘플링
            for (float w = -max_angular; w <= max_angular; w += 0.1) { // 각속도 샘플링
                velocity_samples.push_back({v, w});
            }
        }

        float best_v = 0.0, best_w = 0.0;
        float best_cost = std::numeric_limits<float>::max();

        for (auto [v, w] : velocity_samples) {
            float heading_cost = goal_distance_score(v, w);
            float clearance_cost = obstacle_distance_score(v, w);
            float velocity_cost = max_linear - v;  // 속도가 빠를수록 가중치 추가

            float total_cost = heading_cost * alpha + clearance_cost * beta + velocity_cost * gamma;

            if (total_cost < best_cost) {
                best_cost = total_cost;
                best_v = v;
                best_w = w;
            }
        }

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = best_v;
        cmd_vel.angular.z = best_w;

        pub_cmd->publish(cmd_vel);
    }

float CmdPublisher::goal_distance_score(float v, float w) {
    float dx = goal_.point.x;
    float dy = goal_.point.y;
    float target_angle = std::atan2(dy, dx);
    float heading_cost = std::abs(target_angle - w);
    return heading_cost;
}

float CmdPublisher::obstacle_distance_score(float v, float w) {
    if (nearest_obstacle_distance_ < 0.5) return std::numeric_limits<float>::max();
    return 1.0 / nearest_obstacle_distance_;  // 장애물 가까울수록 비용 증가
}