#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <algorithm>
#include "map.h"

struct astarNode {
    float x, y;
    float g_cost, h_cost;
    astarNode* parent;

    astarNode(float x, float y, float g_cost, float h_cost, astarNode* parent = nullptr) : x(x), y(y), g_cost(g_cost), h_cost(h_cost), parent(parent) {}
    float f_cost() const { return g_cost + h_cost; }

    bool operator>(const astarNode& other) const {
        return f_cost() > other.f_cost();
    }

};
struct astarNodeHash {
    std::size_t operator()(const astarNode& node) const {
        return std::hash<float>()(node.x) ^ std::hash<float>()(node.y);
    }
};

// Define an equality function for the astarNode structure
struct astarNodeEqual {
    bool operator()(const astarNode& lhs, const astarNode& rhs) const {
        return lhs.x == rhs.x && lhs.y == rhs.y;
    }
};

// Define the heuristic function (Euclidean distance)
float heuristic(float x1, float y1, float x2, float y2) {
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

class CmdPublisher : public rclcpp::Node {
public:
    CmdPublisher();
private:
    void timer_tf_callback();

    void timer_cmd_callback();

    void octomap_callback(const OctomapMsg& octomap_msg);

    std::vector<std::vector<int>> CmdPublisher::create_grid_map_from_octomap(const octomap::OcTree& octree);

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    std::vector<std::array<float, 3>> a_star_planner(float start_x, float start_y, float goal_x, float goal_y, const std::vector<std::vector<int>>& grid);

    void divide_path_into_waypoints(const std::vector<std::array<float, 3>>& path, int num_waypoints);
    //dwa
    void dwa_algorithm(int path_index);

    float goal_distance_score(float v, float w);

    float obstacle_distance_score(float nearest_obstacle_distance);

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void goal_orientation_achievement();

    float deviation_from_line_cost(float v, float w);

    float normalize_angle(float angle);
    //geometry
    float x,y,z;
    double pose_x, pose_y, pose_z, pose_w; //added by idiot
    double yaw;
    int current_waypoint_index = 0;
    std::vector<std::vector<int>> grid;
    std::vector<std::array<double,3>> waypoints[10];

    geometry_msgs::msg::PoseStamped goal;
    //flags
    bool goal_received = false;
    bool position_updated = false;
    enum class GoalState {
        NOT_ACHIEVED,
        POSITION_ACHIEVED,
        ACHIEVED
    } goal_state = GoalState::ACHIEVED;
    //sensor
    float lidar_value[360] = {0.f,};
    //Personal Parameter
    const float alpha = 2.5f;
    const float beta = 1.5f;
    const float gamma = 1.2f;
    const float delta = 1.5f;

    const float OBSTACLE_THRESHOLD = 0.25f;
    const float GOAL_THRESHOLD = 0.05f;
    const double max_linear = 0.3f;
    const double max_angular = 0.8f;
    const float Kp = 1.5f;
    const float Ki = 0.1f;
    const float Kd = 0.2f;
    double prev_error = 0.0;
    double integral = 0.0;

    // Grid map parameters
    float grid_resolution = 0.1; // 10 cm per grid cell
    int grid_width = 5; // 5 meters
    int grid_height = 5; // 5 meters
    uint8_t path_index = 0;
    Map map;
    //messages
    rclcpp::TimerBase::SharedPtr timer_cmd, timer_tf;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;

    rclcpp::Subscription<OctomapMsg>::SharedPtr sub_octomap;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal;    

    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

};
