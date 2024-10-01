#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class Deadman : public rclcpp::Node {
    private:
        std::string name;
        std::string topic_prefix_param;
        std::string joy_topic;
        int deadman_button;

        rclcpp::TimerBase::SharedPtr deadman_timer;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

    public:
        Deadman(): Node("deadman",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ) {
            try {
                name = this->get_parameter("name").as_string(); 
                topic_prefix_param = this->get_parameter("topic_prefix").as_string();
            } catch (...) {
                name = "deadman";
                topic_prefix_param = "/fb";
            }

            try{
                rclcpp::Parameter joy_param = this->get_parameter("joy_topic");
                joy_topic = joy_param.as_string();
                rclcpp::Parameter deadman_button_param = this->get_parameter("deadman_button");
                deadman_button = deadman_button_param.as_int();
            } catch(const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Could not find parameters: joy_topic, deadman_button, using defaults");
                joy_topic = topic_prefix_param + "/joy";
                deadman_button = 5;
            }

            deadman_timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Deadman::deadman_timer_callback, this));
            joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(joy_topic, 10, std::bind(&Deadman::joy_callback, this, std::placeholders::_1));
        }
    private:
        void deadman_timer_callback(){
            RCLCPP_INFO(this->get_logger(), "Deadman timer");
        }

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
            if (msg->buttons[deadman_button] == 1) {
                RCLCPP_INFO(this->get_logger(), "Deadman pressed");
            }
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