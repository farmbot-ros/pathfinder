#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "std_srvs/srv/trigger.hpp"

class Deadman : public rclcpp::Node {
    private:
        std::string name;
        std::string topic_prefix_param;
        std::string joystick_topic;
        int deadman_button;
        std::string start_topic_param;
        std::string pause_topic_param;

        rclcpp::TimerBase::SharedPtr deadman_timer;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
        int button_state = 0;

        //service client
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_srv_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_srv_;

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

            joystick_topic = topic_prefix_param + "/joy";
            deadman_button = 4;
            try{
                rclcpp::Parameter joy_param = this->get_parameter("joystick_topic");
                joystick_topic = joy_param.as_string();
                rclcpp::Parameter deadman_button_param = this->get_parameter("deadman_button");
                deadman_button = deadman_button_param.as_int();
            } catch(const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Could not find parameters: joy_topic, deadman_button, using defaults");
            }

            start_topic_param = topic_prefix_param + "/nav/start";
            pause_topic_param = topic_prefix_param + "/nav/pause";
            try {
                rclcpp::Parameter start_topic = this->get_parameter("start_topic");
                start_topic_param = start_topic.as_string();
                rclcpp::Parameter pause_topic = this->get_parameter("pause_topic");
                pause_topic_param = pause_topic.as_string();
            } catch(const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Could not find parameters: start_topic, pause_topic, using defaults");
            }

            deadman_timer = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&Deadman::deadman_timer_callback, this));
            joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(joystick_topic, 10, std::bind(&Deadman::joy_callback, this, std::placeholders::_1));
        
            start_srv_ = this->create_client<std_srvs::srv::Trigger>(start_topic_param);
            pause_srv_ = this->create_client<std_srvs::srv::Trigger>(pause_topic_param);
        }
    private:
        void deadman_timer_callback(){
            RCLCPP_INFO_ONCE(this->get_logger(), "Deadman timer is running");
            // deadman_timer->reset();
            auto wait = std::chrono::seconds(1);
            while (!start_srv_->wait_for_service(wait) && !pause_srv_->wait_for_service(wait)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "services %s and %s not available, waiting again...", start_srv_->get_service_name(), pause_srv_->get_service_name());
            }

            if (button_state == 1) {
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                start_srv_->async_send_request(request);
            } else {
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                pause_srv_->async_send_request(request);
            }
        }

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
            if (msg->buttons[deadman_button] == 1) {
                button_state = 1;
            } else {
                button_state = 0;
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