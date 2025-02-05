#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "map.h"

class CmdPublisher : public rclcpp::Node {
public:
    CmdPublisher();

private:
    void timer_tf_callback();

    void timer_cmd_callback();

    void octomap_callback(const OctomapMsg& octomap_msg);

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    //dwa
    void velocity_by_dwa();

    float goal_distance_score(float v, float w);

    float obstacle_distance_score(float v, float w);

    //geometry
    double x,y,z;
    double pose_x, pose_y, pose_z; //added by idiot
    bool position_updated = false;
    //sensor
    float lidar_value[360] = {0.f,};

    //Personal Parameter
    const float alpha = 1.f;
    const float beta = 2.f;
    const float gamma = 0.5f;
    const float OBSTACLE_THRESHOLD = 0.1f;
    const float max_linear = 0.f;
    const float max_angular = 0.f;
    const float Kp = 1.f;
    const float Ki = 0.f;
    const float Kd = 0.f;
    Map map;
    //messages
    rclcpp::TimerBase::SharedPtr timer_cmd, timer_tf;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;

    rclcpp::Subscription<OctomapMsg>::SharedPtr sub_octomap;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
};
