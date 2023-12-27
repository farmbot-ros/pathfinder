#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "farmbot_interfaces/action/nav.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
 
#include <iostream>
#include <thread>
#include <mutex>
#include <string>


class Navigator : public rclcpp::Node {
    private:
        bool inited_waypoints = false;
        message_filters::Subscriber<sensor_msgs::msg::NavSatFix> fix_sub_;
        message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, nav_msgs::msg::Odometry>>> sync_;
        
        nav_msgs::msg::Path path_nav;
        rclcpp::TimerBase::SharedPtr path_timer;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

        geometry_msgs::msg::Pose current_pose_;
        sensor_msgs::msg::NavSatFix current_gps_;
        geometry_msgs::msg::Point target_pose_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;
        //action_server
        using TheAction = farmbot_interfaces::action::Nav;
        using GoalHandle = rclcpp_action::ServerGoalHandle<TheAction>;
        std::shared_ptr<GoalHandle> handeler_;
        rclcpp_action::Server<TheAction>::SharedPtr action_server_;
    public:
        Navigator(): Node("path_server",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ) {
            this->action_server_ = rclcpp_action::create_server<TheAction>(this, "/navigation",
                std::bind(&Navigator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&Navigator::handle_cancel, this, std::placeholders::_1),
                std::bind(&Navigator::handle_accepted, this, std::placeholders::_1)
            );

            std::string name = "path_server";
            std::string topic_prefix_param = "/fb";
            try {
                std::string name = this->get_parameter("name").as_string(); 
                std::string topic_prefix_param = this->get_parameter("topic_prefix").as_string();
            } catch (...) {
                RCLCPP_INFO(this->get_logger(), "No parameters found, using default values");
            }

            path_timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Navigator::path_timer_callback, this));
            fix_sub_.subscribe(this, topic_prefix_param + "/loc/fix");
            odom_sub_.subscribe(this, topic_prefix_param + "/loc/odom");
            sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, nav_msgs::msg::Odometry>>>(10);
            sync_->connectInput(fix_sub_, odom_sub_);
            sync_->registerCallback(std::bind(&Navigator::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

            path_pub = this->create_publisher<nav_msgs::msg::Path>(topic_prefix_param + "/nav/path", 10);
            cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        }
    
    private:
        void sync_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& fix, const nav_msgs::msg::Odometry::ConstSharedPtr& odom) {
            // RCLCPP_INFO(this->get_logger(), "Sync callback");
            current_pose_ = odom->pose.pose;
            current_gps_ = *fix;
        }

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TheAction::Goal> goal){
            goal->initial_path.poses;
            RCLCPP_INFO(this->get_logger(), "Received goal request");
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle){
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle){
            if (handeler_ && handeler_->is_active()) {
                RCLCPP_INFO(this->get_logger(), "ABORTING PREVIOUS GOAL...");
                path_nav.poses.clear();
                stop_moving();
                handeler_->abort(std::make_shared<TheAction::Result>());
            }
            handeler_ = goal_handle;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&Navigator::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandle> goal_handle){
            // RCLCPP_INFO(this->get_logger(), "Executing goal");
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<TheAction::Feedback>();
            auto result = std::make_shared<TheAction::Result>();
            rclcpp::Rate loop_rate(10);

            if (goal->abort.data) {
                stop_moving();
                result->plan_result = std_msgs::msg::Empty();
                goal_handle->abort(result);
                return;
            }
            for (auto a_pose: path_setter(goal->initial_path.poses)) {
                target_pose_ = a_pose.pose.position;
                RCLCPP_INFO(this->get_logger(), "Going to: %f, %f, currently at: %f, %f", target_pose_.x, target_pose_.y, current_pose_.position.x, current_pose_.position.y);
                while (rclcpp::ok()){
                    if (goal_handle->is_canceling()) {
                        result->plan_result = std_msgs::msg::Empty();
                        path_nav.poses.clear();
                        stop_moving();
                        goal_handle->canceled(result);
                        return;
                    } else if (!goal_handle->is_active()){
                        result->plan_result = std_msgs::msg::Empty();
                        stop_moving();
                        return;
                    }
                    std::array<double, 3> nav_params = get_nav_params();
                    geometry_msgs::msg::Twist twist;
                    twist.linear.x = nav_params[0];
                    twist.angular.z = nav_params[1];
                    cmd_vel->publish(twist);
                    RCLCPP_INFO(this->get_logger(), "Twist: %f, %f", twist.linear.x, twist.angular.z);
                    goal_handle->publish_feedback(feedback);
                    loop_rate.sleep();
                    if (nav_params[2] < 0.1) {
                        break;
                    }
                }
            }
            // Check if goal is done
            if (rclcpp::ok()) {
                result->plan_result = std_msgs::msg::Empty();
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
        }
        
        std::array<double, 3> get_nav_params(double angle_max=0.4, double velocity_max=0.3) {
            double distance = std::sqrt(
                std::pow(target_pose_.x - current_pose_.position.x, 2) + 
                std::pow(target_pose_.y - current_pose_.position.y, 2));
            double velocity = 0.2 * distance;
            double preheading = std::atan2(
                target_pose_.y - current_pose_.position.y, 
                target_pose_.x - current_pose_.position.x);
            double orientation = std::atan2(2 * (current_pose_.orientation.w * current_pose_.orientation.z + 
                                                current_pose_.orientation.x * current_pose_.orientation.y), 
                                            1 - 2 * (std::pow(current_pose_.orientation.y, 2) + std::pow(current_pose_.orientation.z, 2)));
            double heading = preheading - orientation;
            double heading_corrected = std::atan2(std::sin(heading), std::cos(heading));
            double angular = std::max(-angle_max, std::min(heading_corrected, angle_max));
            velocity = std::max(-velocity_max, std::min(velocity, velocity_max));
            return {velocity, angular, distance};
        }
        void stop_moving() {
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            cmd_vel->publish(twist);
        }
        std::vector<geometry_msgs::msg::PoseStamped> path_setter(const std::vector<geometry_msgs::msg::PoseStamped>& poses){
            path_nav.poses.clear();
            for (const geometry_msgs::msg::PoseStamped& element : poses) {
                geometry_msgs::msg::PoseStamped a_pose = element;
                a_pose.header.stamp = this->now();
                a_pose.header.frame_id = "map";
                path_nav.poses.push_back(a_pose);
            }
            inited_waypoints = true;
            return path_nav.poses;
        }
        void path_timer_callback(){
            path_nav.header.stamp = this->now();
            path_nav.header.frame_id = "map";
            if (inited_waypoints){
                path_pub->publish(path_nav);
            }
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto navfix = std::make_shared<Navigator>();
    rclcpp::spin(navfix);
    rclcpp::shutdown();
    return 0;
}