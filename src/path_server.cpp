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

#include "farmbot_interfaces/action/waypoints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
 
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
        
        // nav_msgs::msg::Path path_nav;
        std::string status;
        farmbot_interfaces::msg::Waypoints path_nav;
        rclcpp::TimerBase::SharedPtr path_timer;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

        geometry_msgs::msg::Pose current_pose_;
        sensor_msgs::msg::NavSatFix current_gps_;
        geometry_msgs::msg::Twist current_twist_;
        geometry_msgs::msg::Point target_pose_;
        std::string current_uuid_ = "ZER0";
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;
        //action_server
        using TheAction = farmbot_interfaces::action::Waypoints;
        using GoalHandle = rclcpp_action::ServerGoalHandle<TheAction>;
        std::shared_ptr<GoalHandle> handeler_;
        rclcpp_action::Server<TheAction>::SharedPtr action_server_;
        //services
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv;
        
    public:
        Navigator(): Node("path_server",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ) {
            this->action_server_ = rclcpp_action::create_server<TheAction>(this, "/nav/mission",
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

            //services
            start_srv = this->create_service<std_srvs::srv::Trigger>(topic_prefix_param + "/nav/start", std::bind(&Navigator::start_callback, this, std::placeholders::_1, std::placeholders::_2));
            pause_srv = this->create_service<std_srvs::srv::Trigger>(topic_prefix_param + "/nav/pause", std::bind(&Navigator::pause_callback, this, std::placeholders::_1, std::placeholders::_2));
            stop_srv = this->create_service<std_srvs::srv::Trigger>(topic_prefix_param + "/nav/stop", std::bind(&Navigator::stop_callback, this, std::placeholders::_1, std::placeholders::_2));
        }
    
    private:
        void sync_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& fix, const nav_msgs::msg::Odometry::ConstSharedPtr& odom) {
            // RCLCPP_INFO(this->get_logger(), "Sync callback");
            current_pose_ = odom->pose.pose;
            current_gps_ = *fix;
        }

        void start_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
            (void)request;
            status = "running";
            response->success = true;
            response->message = "Started";
        }

        void pause_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
            (void)request;
            status = "paused";
            response->success = true;
            response->message = "Paused";
        }

        void stop_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
            (void)request;
            status = "stopped";
            response->success = true;
            response->message = "Stopped";
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

            for (auto a_pose: path_setter(goal->initial_path.poses)) {
                target_pose_ = a_pose.pose.position;
                RCLCPP_INFO(this->get_logger(), "Going to: %f, %f, currently at: %f, %f", target_pose_.x, target_pose_.y, current_pose_.position.x, current_pose_.position.y);
                while (rclcpp::ok()){
                    if (goal_handle->is_canceling()) {
                        result->success = std_msgs::msg::Bool();
                        path_nav.poses.clear();
                        stop_moving();
                        goal_handle->canceled(result);
                        return;
                    } else if (!goal_handle->is_active()){
                        result->success = std_msgs::msg::Bool();
                        stop_moving();
                        return;
                    }
                    std::array<double, 3> nav_params = get_nav_params();
                    geometry_msgs::msg::Twist twist;
                    twist.linear.x = nav_params[0];
                    twist.angular.z = nav_params[1];
                    current_twist_ = twist;
                    cmd_vel->publish(twist);
                    // RCLCPP_INFO(this->get_logger(), "Twist: %f, %f", twist.linear.x, twist.angular.z);
                    fill_feedback(feedback, current_uuid_);
                    RCLCPP_INFO(this->get_logger(), "Pose: (%f, %f), GPS: (%f, %f), Target: (%f, %f), Distance: %f, Target UUID: %s", 
                        current_pose_.position.x, 
                        current_pose_.position.y, 
                        current_gps_.latitude, 
                        current_gps_.longitude, 
                        target_pose_.x, 
                        target_pose_.y, 
                        nav_params[2], 
                        std::string(a_pose.uuid.data).c_str()
                    );
                    goal_handle->publish_feedback(feedback);
                    loop_rate.sleep();
                    if (nav_params[2] < 0.1) {
                        break;
                    }
                }
                current_uuid_ = a_pose.uuid.data;
            }
            // Goal is done, send success message
            if (rclcpp::ok()) {
                auto message = std_msgs::msg::Bool();
                message.data = true;
                result->success = message;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
        }

        void fill_feedback(TheAction::Feedback::SharedPtr feedback, std::string uuid="00"){
            feedback->pose = current_pose_;
            feedback->gps = current_gps_;
            feedback->last_uuid.data = uuid;
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
        std::vector<farmbot_interfaces::msg::Waypoint> path_setter(const std::vector<farmbot_interfaces::msg::Waypoint>& poses){
            path_nav.poses.clear();
            for (const farmbot_interfaces::msg::Waypoint& element : poses) {
                farmbot_interfaces::msg::Waypoint a_pose = element;
                a_pose.header.stamp = this->now();
                a_pose.header.frame_id = "map";
                path_nav.poses.push_back(a_pose);
            }
            inited_waypoints = true;
            return path_nav.poses;
        }

        nav_msgs::msg::Path waypoint_to_path(const std::vector<farmbot_interfaces::msg::Waypoint>& poses){
            nav_msgs::msg::Path path;
            path.header.stamp = this->now();
            path.header.frame_id = "map";
            for (const farmbot_interfaces::msg::Waypoint& element : poses) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp = this->now();
                pose.header.frame_id = "map";
                pose.pose = element.pose;
                path.poses.push_back(pose);
            }
            return path;
        }

        void path_timer_callback(){
            path_nav.header.stamp = this->now();
            path_nav.header.frame_id = "map";
            if (inited_waypoints){
                path_pub->publish(waypoint_to_path(path_nav.poses));
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