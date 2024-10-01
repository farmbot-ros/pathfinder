#include <rclcpp/rclcpp.hpp>

class Deadman : public rclcpp::Node {
    public:
        Deadman(): Node("deadman") {
            RCLCPP_INFO(this->get_logger(), "Deadman node started");
        }
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<Deadman>();
    try {
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), e.what());
    }
    rclcpp::shutdown();
    return 0;
}