#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <algorithm>
#include "map.h"

struct PIDController {
  double Kp, Ki, Kd;
  double reference = 0.f;
  double current = 0.f;
  double dt = 0.1f;
  double derivative;
  double prev_error, integral;
  double error;

  float Output(double reference_, double current_=0.f, bool normalize=false) {
    reference = reference_;
    current = current_;
    error = reference - current;
    integral += error;
    derivative = (error - prev_error) / dt;
    prev_error = error;
    if(normalize) return Kp * atan2(sin(error), cos(error)) + Ki * atan2(sin(integral), cos(integral)) + Kd * atan2(sin(derivative), cos(derivative)); 
    else return Kp * error + Ki * integral + Kd * derivative;
  }

  void SetPID(double Kp_, double Ki_, double Kd_) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
  }

  void ErrorInit() {
    prev_error = 0.0;
    integral = 0.0;
    derivative = 0.0;
  }

};

struct AstarNode {
  float x, y;
  float g, h; 
  std::shared_ptr<AstarNode> parent;
  float cost() const { return g + h; }
};

struct Compare {
  bool operator()(const AstarNode& a, const AstarNode& b) {
      if( a.cost() ==b.cost()){
        return a.g > b.g; 
      }
      return a.cost() > b.cost();
  }
};


class CmdPublisher : public rclcpp::Node {
  public:
    CmdPublisher();
  private:
    std::vector<geometry_msgs::msg::PoseStamped> plan_path(float start_x, float start_y, float goal_x, float goal_y);

    std::pair<int, int> to_grid(float x, float y, float resolution);

    float heuristic(float x1, float y1, float x2, float y2);

    float distance(const AstarNode &a, const AstarNode &b);

    bool is_goal(float x, float y, float goal_x, float goal_y);

    bool is_goal_path(float x, float y, float goal_x, float goal_y);

    std::vector<AstarNode> get_neighbors(const AstarNode &current);

    std::vector<geometry_msgs::msg::PoseStamped> reconstruct_path(const AstarNode &goal_node);

    void waypoint_visualize(const std::vector<geometry_msgs::msg::PoseStamped>& path);

    void timer_tf_callback();

    void timer_cmd_callback();
    
    void clear_obstacle();

    void octomap_callback(const OctomapMsg& octomap_msg);

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void Controller();

    float normalize_angle(float angle);
    //geometry
    float x,y,z;
    double pose_x, pose_y, pose_z, pose_w; //added by idiot
    double yaw;
    
    octomap::point3d start_point;
    geometry_msgs::msg::PoseStamped goal;
    geometry_msgs::msg::PoseStamped prev_target;
    std::vector<geometry_msgs::msg::PoseStamped> path;
    //flags
    bool goal_received = false;

    bool position_updated = false;

    enum class GoalState {
        NOT_ACHIEVED,
        POSITION_ACHIEVED,
        ACHIEVED
    } goal_state = GoalState::ACHIEVED;
    bool evasion = false;
    bool deviation = false;
    //sensor
    float minimum_distance = std::numeric_limits<float>::max();
    float minimum_distance_angle = 0.f;
    float range_max = 0.f;
    float range_min = 0.f;
    float lidar_value[360] = {0.f,};
    float angle_increment = 0.f;

    float resolution = 0.2;
    const float OBSTACLE_THRESHOLD = 0.3f;
    const float GOAL_THRESHOLD = 0.1f;
    float search_radius = 10.0;
    //PID
    PIDController linear;
    PIDController angular;

    const float max_linear = 0.15f;
    const float max_angular = 0.8f;

    Map map;
    //messages
    rclcpp::TimerBase::SharedPtr timer_cmd, timer_tf;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker;

    rclcpp::Subscription<OctomapMsg>::SharedPtr sub_octomap;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal;    

    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

};
