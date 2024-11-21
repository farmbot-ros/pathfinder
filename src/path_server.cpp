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
#include "farmbot_interfaces/action/control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include <iostream>
#include <thread>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

enum class RobotState {
    Idle,
    Running,
    Paused,
    Stopped
};

std::string stateToString(RobotState state) {
    switch (state) {
        case RobotState::Idle: return "Idle";
        case RobotState::Running: return "Running";
        case RobotState::Paused: return "Paused";
        case RobotState::Stopped: return "Stopped";
        default: return "Unknown";
    }
}

float deg2rad(float deg) {
    return deg * M_PI / 180;
}

float rad2deg(float rad) {
    return rad * 180 / M_PI;
}


class Navigator : public rclcpp::Node {
    private:
        bool inited_waypoints = false;
        message_filters::Subscriber<sensor_msgs::msg::NavSatFix> fix_sub_;
        message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, nav_msgs::msg::Odometry>>> sync_;

        rclcpp::Time initial_time;
        std::string name;
        std::string namespace_;
        bool autostart;

        // nav_msgs::msg::Path path_nav;
        RobotState state;
        farmbot_interfaces::msg::Waypoints path_nav;
        rclcpp::TimerBase::SharedPtr path_timer;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

        bool controller_running;
        float distance_to_target;
        geometry_msgs::msg::Pose current_pose_;
        geometry_msgs::msg::Pose target_pose_;
        sensor_msgs::msg::NavSatFix current_gps_;
        geometry_msgs::msg::Twist send_twist_;
        std::string current_uuid_ = "ZER0";
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;
        //action_server
        using TheAction = farmbot_interfaces::action::Waypoints;
        using GoalHandle = rclcpp_action::ServerGoalHandle<TheAction>;
        std::shared_ptr<GoalHandle> handeler_;
        rclcpp_action::Server<TheAction>::SharedPtr action_server_;
        //action_client
        rclcpp_action::Client<farmbot_interfaces::action::Control>::SharedPtr control_client_;

        //services
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv;        //Diagnostic Updater

        //diagnostic
        diagnostic_updater::Updater updater_;
        diagnostic_msgs::msg::DiagnosticStatus status;
        rclcpp::TimerBase::SharedPtr diagnostic_timer_;

    public:
        Navigator(): Node("path_server",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ), updater_(this) {
            name = this->get_parameter_or<std::string>("name", "path_server");
            autostart = this->get_parameter_or<bool>("autostart", true);
            state = RobotState::Idle;
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "Not initialized";
            //action server
            this->action_server_ = rclcpp_action::create_server<TheAction>(this, "nav/mission",
                std::bind(&Navigator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&Navigator::handle_cancel, this, std::placeholders::_1),
                std::bind(&Navigator::handle_accepted, this, std::placeholders::_1)
            );
            //action client
            control_client_ = rclcpp_action::create_client<farmbot_interfaces::action::Control>(this, "con/zeroturn");
            // subscribers
            fix_sub_.subscribe(this, "loc/fix");
            odom_sub_.subscribe(this, "loc/odom");
            sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, nav_msgs::msg::Odometry>>>(10);
            sync_->connectInput(fix_sub_, odom_sub_);
            sync_->registerCallback(std::bind(&Navigator::sync_callback, this, std::placeholders::_1, std::placeholders::_2));
            //publishers
            path_pub = this->create_publisher<nav_msgs::msg::Path>("nav/path", 10);
            cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            //services
            start_srv = this->create_service<std_srvs::srv::Trigger>("nav/start", std::bind(&Navigator::start_callback, this, std::placeholders::_1, std::placeholders::_2));
            pause_srv = this->create_service<std_srvs::srv::Trigger>("nav/pause", std::bind(&Navigator::pause_callback, this, std::placeholders::_1, std::placeholders::_2));
            stop_srv = this->create_service<std_srvs::srv::Trigger>("nav/stop", std::bind(&Navigator::stop_callback, this, std::placeholders::_1, std::placeholders::_2));
            //timer
            path_timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Navigator::timer_callback, this));

            namespace_ = this->get_namespace();
            if (!namespace_.empty() && namespace_[0] == '/') {
                namespace_ = namespace_.substr(1); // Remove leading slash
            }

            // Diagnostic Updater
            updater_.setHardwareID(static_cast<std::string>(this->get_namespace()) + "/nav");
            updater_.add("Navigation Status", this, &Navigator::check_system);
            diagnostic_timer_ = this->create_wall_timer(1s, std::bind(&Navigator::diagnostic_callback, this));
        }

    private:
        void diagnostic_callback() {
            updater_.force_update();
        }

        void check_system(diagnostic_updater::DiagnosticStatusWrapper &stat) {
            stat.summary(status.level, status.message);
        }

        void sync_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& fix, const nav_msgs::msg::Odometry::ConstSharedPtr& odom) {
            // RCLCPP_INFO(this->get_logger(), "Sync callback");
            current_pose_ = odom->pose.pose;
            current_gps_ = *fix;
        }

        void timer_callback() {
            path_nav.header.stamp = this->now();
            path_nav.header.frame_id = namespace_ + "/map";
            if (inited_waypoints) {
                nav_msgs::msg::Path path;
                path.header.stamp = this->now();
                path.header.frame_id = namespace_ + "/map";
                for (const farmbot_interfaces::msg::Waypoint& element : path_nav.poses) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.stamp = this->now();
                    pose.header.frame_id = namespace_ + "/map";
                    pose.pose = element.pose;
                    path.poses.push_back(pose);
                }
                path_pub->publish(path);
            }
        }

        void start_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
            (void)request;
            state = RobotState::Running;
            response->success = true;
            response->message = "Started";
        }

        void pause_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
            (void)request;
            state = RobotState::Paused;
            response->success = true;
            response->message = "Paused";
        }

        void stop_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
            (void)request;
            state = RobotState::Stopped;
            response->success = true;
            response->message = "Stopped";
        }

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TheAction::Goal> goal){
            goal->mission.poses;
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
            initial_time = this->now();
            // RCLCPP_INFO(this->get_logger(), "Executing goal");
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<TheAction::Feedback>();
            auto result = std::make_shared<TheAction::Result>();

            rclcpp::Rate wait_rate(1000);
            while(state == RobotState::Idle){
                if (autostart) {
                    state = RobotState::Running;
                    break;
                }
                fill_feedback(feedback);
                goal_handle->publish_feedback(feedback);
                wait_rate.sleep();
                RCLCPP_INFO(this->get_logger(), "Waiting for start signal");
            }

            rclcpp::Rate loop_rate(10);
            std::vector<farmbot_interfaces::msg::Waypoint> the_path = path_setter(goal->mission.poses);
            //if the first point is 1 meter to current position, skip it
            if (the_path.size() > 0) {
                target_pose_ = the_path[0].pose;
                double dx = target_pose_.position.x - current_pose_.position.x;
                double dy = target_pose_.position.y - current_pose_.position.y;
                if (std::hypot(dx, dy) < 1.0) {
                    the_path.erase(the_path.begin());
                }
            }
            for (auto a_pose: the_path) {
                target_pose_ = a_pose.pose;
                RCLCPP_INFO(this->get_logger(), "Going to: %f, %f, currently at: %f, %f", target_pose_.position.x, target_pose_.position.y, current_pose_.position.x, current_pose_.position.y);
                controller_running = send_control_goal(target_pose_);
                while (rclcpp::ok() && controller_running) {
                    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                    status.message = "Moving to goal";
                    if (goal_handle->is_canceling()) {
                        fill_result_n_stop(result, false);
                        goal_handle->canceled(result);
                        return;
                    } else if (!goal_handle->is_active()){
                        fill_result_n_stop(result, false);
                        return;
                    }

                    if (state == RobotState::Paused) {
                        stop_moving();
                        while (state == RobotState::Paused) {
                            fill_feedback(feedback, current_uuid_);
                            goal_handle->publish_feedback(feedback);
                            wait_rate.sleep();
                        }
                    } else if (state == RobotState::Stopped) {
                        // stop_moving();
                        fill_result_n_stop(result, false);
                        goal_handle->abort(result);
                        // return;
                    }
                    cmd_vel->publish(send_twist_);
                    fill_feedback(feedback, current_uuid_);
                    goal_handle->publish_feedback(feedback);
                    loop_rate.sleep();
                }
                current_uuid_ = a_pose.uuid.data;
            }
            // Goal is done, send success message
            if (rclcpp::ok()) {
                fill_result_n_stop(result);
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
        }

        void fill_feedback(TheAction::Feedback::SharedPtr feedback, std::string uuid="00"){
            feedback->pose = current_pose_;
            feedback->gps = current_gps_;
            feedback->last_uuid.data = uuid;
        }

        void fill_result_n_stop(TheAction::Result::SharedPtr result, bool success=true){
            result->success.data = success;
            result->time_it_took = this->now() - initial_time;
            path_nav.poses.clear();
            stop_moving();
            cancel_control_goal();
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
                a_pose.header.frame_id = namespace_ + "/map";
                path_nav.poses.push_back(a_pose);
            }
            inited_waypoints = true;
            return path_nav.poses;
        }

        bool send_control_goal(const geometry_msgs::msg::Pose& target_pose) {
            using ActionGoalHandle = rclcpp_action::ClientGoalHandle<farmbot_interfaces::action::Control>;
            // Wait for the action server to be ready
            while (!control_client_->wait_for_action_server(std::chrono::seconds(1)) && rclcpp::ok()) {
                RCLCPP_INFO(this->get_logger(), "Waiting for the control action server to be ready...");
            }
            // Create the goal message
            auto goal_msg = farmbot_interfaces::action::Control::Goal();
            goal_msg.segment.destination.pose = target_pose;
            // Send the goal and specify callbacks
            auto send_goal_options = rclcpp_action::Client<farmbot_interfaces::action::Control>::SendGoalOptions();
            // Specify a callback for when the goal is accepted by the server
            send_goal_options.goal_response_callback = [this](std::shared_ptr<ActionGoalHandle> goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                }
            };
            // Specify a callback for when feedback is received
            send_goal_options.feedback_callback = [this](ActionGoalHandle::SharedPtr, const std::shared_ptr<const farmbot_interfaces::action::Control::Feedback> feedback) {
                // RCLCPP_INFO(this->get_logger(), "Received feedback: linear=%f, angular=%f", feedback->twist.linear.x, feedback->twist.angular.z);
                controller_running = true;
                send_twist_ = feedback->twist;
                distance_to_target = feedback->distance.data;
            };
            // Specify a callback for when the goal is complete
            send_goal_options.result_callback = [this](const ActionGoalHandle::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Control succeeded");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_INFO(this->get_logger(), "Control was aborted");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(this->get_logger(), "Control was canceled");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        break;
                }
                controller_running = false;
            };
            // Send the goal
            control_client_->async_send_goal(goal_msg, send_goal_options);
            RCLCPP_INFO(this->get_logger(), "Sending control goal to (%f, %f)", target_pose.position.x, target_pose.position.y);
            return true;
        }

        void cancel_control_goal() {
            if (control_client_->wait_for_action_server(std::chrono::seconds(1))) {
                control_client_->async_cancel_all_goals();
            }
        }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<Navigator>();
    try {
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), e.what());
    }
    rclcpp::shutdown();
    return 0;
}
