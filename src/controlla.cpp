#include "controlla.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

CmdPublisher::CmdPublisher() : Node("cmd_publisher") {
    // Publisher
    pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // Subscriber
    sub_octomap = this->create_subscription<OctomapMsg>("octomap_full", 10, std::bind(&CmdPublisher::octomap_callback, this, _1));
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
        return;
    }

    octomap::point3d search_point(x,y,z);
    octomap::point3d closest_obstacle;
    float distance;

    map.get_distance_and_closest_obstacle(search_point, distance, closest_obstacle);
    RCLCPP_INFO_STREAM(this->get_logger(), "Distance to obstacle: " << std::to_string(distance) << ", Closest obsatcle: " << closest_obstacle);

    geometry_msgs::msg::Twist cmd_vel;
    pub_cmd->publish(cmd_vel);
}

void CmdPublisher::octomap_callback(const OctomapMsg& octomap_msg) {
    octomap::point3d world_min(-5, -5, 0);
    octomap::point3d world_max(5, 5, 2);  
    map.update(octomap_msg, world_min, world_max);
}