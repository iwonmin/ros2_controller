#include "controlla.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavPublisher>());
    rclcpp::shutdown();
    return 0;
}