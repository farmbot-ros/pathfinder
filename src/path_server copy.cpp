#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "farmbot_interfaces/action/nav.hpp"
#include "farmbot_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
 
#include <iostream>
#include <thread>
#include <mutex>
#include <string>

class Locator : public rclcpp::Node {
    private:
        int* numberPtr;
        bool* inited_waypoints;
        geometry_msgs::msg::Pose* pose;
        sensor_msgs::msg::NavSatFix* gps;
        std::vector<geometry_msgs::msg::PoseStamped>* points;

        nav_msgs::msg::Path path;
        message_filters::Subscriber<sensor_msgs::msg::NavSatFix> fix_sub_;
        message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, nav_msgs::msg::Odometry>>> sync_;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

        std::mutex mtx;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr path_timer;

    public:
        Locator() : Node("path_server"){
            std::string name = "path_server";
            std::string topic_prefix_param = "/fb";
            try {
                std::string name = this->get_parameter("name").as_string(); 
                std::string topic_prefix_param = this->get_parameter("topic_prefix").as_string();
            } catch (...) {
                RCLCPP_INFO(this->get_logger(), "No parameters found, using default values");
            }

            fix_sub_.subscribe(this, topic_prefix_param + "/loc/fix");
            odom_sub_.subscribe(this, topic_prefix_param + "/loc/odom");
            sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, nav_msgs::msg::Odometry>>>(10);
            sync_->connectInput(fix_sub_, odom_sub_);
            sync_->registerCallback(std::bind(&Locator::sync_callback, this, std::placeholders::_1, std::placeholders::_2));
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Locator::change_values, this));
            path_timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Locator::path_timer_callback, this));
        }

        void set_values(int* numPtr, bool* inited_waypointsPtr,  geometry_msgs::msg::Pose* posePtr, sensor_msgs::msg::NavSatFix* gpsPtr, std::vector<geometry_msgs::msg::PoseStamped>* pointsPtr ) {
            RCLCPP_INFO(this->get_logger(), "Setting values");
            numberPtr = numPtr;
            inited_waypoints = inited_waypointsPtr;
            pose = posePtr;
            gps = gpsPtr;
            points = pointsPtr;
        }

        void change_values() {
            std::unique_lock<std::mutex> lock(mtx);
            if (numberPtr) {
                (*numberPtr) += 1;
                RCLCPP_INFO(this->get_logger(), "Values changed to %d", *numberPtr);
            }
        }
        void sync_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& fix, const nav_msgs::msg::Odometry::ConstSharedPtr& odom) {
            std::unique_lock<std::mutex> lock(mtx);
            if (gps) {
                gps->latitude = fix->latitude;
                gps->longitude = fix->longitude;
                gps->altitude = fix->altitude;
                // RCLCPP_INFO(this->get_logger(), "GPS changed");
            }
            if (pose) {
                pose->position.x = odom->pose.pose.position.x;
                pose->position.y = odom->pose.pose.position.y;
                pose->position.z = odom->pose.pose.position.z;
                pose->orientation.x = odom->pose.pose.orientation.x;
                pose->orientation.y = odom->pose.pose.orientation.y;
                pose->orientation.z = odom->pose.pose.orientation.z;
                pose->orientation.w = odom->pose.pose.orientation.w;
                // RCLCPP_INFO(this->get_logger(), "Pose changed");
            }
        }
        void path_timer_callback(){
            std::unique_lock<std::mutex> lock(mtx);
            path.header.stamp = this->now();
            path.header.frame_id = "map";
            if (points && *inited_waypoints){
                for (auto &i : *points){
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = path.header;
                    pose.pose.position.x = i.pose.position.x;
                    pose.pose.position.y = i.pose.position.y;
                    path.poses.push_back(pose);
                }
                path_pub->publish(path);
            }
        }
};

class Navigator : public rclcpp::Node {
    private:
        int* numberPtr;
        bool* inited_waypoints;
        geometry_msgs::msg::Pose* pose;
        sensor_msgs::msg::NavSatFix* gps;
        std::vector<geometry_msgs::msg::PoseStamped>* points;

        std::mutex mtx;
        // rclcpp::TimerBase::SharedPtr timer_;

        geometry_msgs::msg::Point current_pose_;
        geometry_msgs::msg::Quaternion current_orientation_;
        geometry_msgs::msg::Point target_pose_;

        //action_server
        using TheAction = farmbot_interfaces::action::Fibonacci;
        using GoalHandle = rclcpp_action::ServerGoalHandle<TheAction>;
        rclcpp_action::Server<TheAction>::SharedPtr action_server_;
    public:
        void set_values(int* numPtr, bool* inited_waypointsPtr,  geometry_msgs::msg::Pose* posePtr, sensor_msgs::msg::NavSatFix* gpsPtr, std::vector<geometry_msgs::msg::PoseStamped>* pointsPtr ) {
            RCLCPP_INFO(this->get_logger(), "Setting values");
            numberPtr = numPtr;
            inited_waypoints = inited_waypointsPtr;
            pose = posePtr;
            gps = gpsPtr;
            points = pointsPtr;
        }

        explicit Navigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("path_server", options) {
            using namespace std::placeholders;
            // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Navigator::readValues, this));

            this->action_server_ = rclcpp_action::create_server<TheAction>(this, "fibonacci",
                std::bind(&Navigator::handle_goal, this, _1, _2),
                std::bind(&Navigator::handle_cancel, this, _1),
                std::bind(&Navigator::handle_accepted, this, _1)
            );
        }
    private:
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TheAction::Goal> goal){
            RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle){
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle){
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&Navigator::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandle> goal_handle){
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            rclcpp::Rate loop_rate(1);
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<TheAction::Feedback>();
            auto & sequence = feedback->partial_sequence;
            sequence.push_back(0);
            sequence.push_back(1);
            auto result = std::make_shared<TheAction::Result>();

            for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
            // Check if there is a cancel request
                if (goal_handle->is_canceling()) {
                    result->sequence = sequence;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }
                // Update sequence
                auto now = sequence[i] + sequence[i - 1];
                sequence.push_back(now);
                
                std::unique_lock<std::mutex> lock(mtx);
                if (numberPtr) {
                    (*numberPtr) = now;
                }
                RCLCPP_INFO(this->get_logger(), "-> Values read as %d", *numberPtr);
                mtx.unlock();
                // Publish feedback
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Publish feedback");
                loop_rate.sleep();
            }

            // Check if goal is done
            if (rclcpp::ok()) {
                result->sequence = sequence;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
        }
        
        std::array<double, 3> get_nav_params(double angle_max=0.4, double velocity_max=0.3) {
            double distance = std::sqrt(
                std::pow(target_pose_.x - current_pose_.x, 2) + 
                std::pow(target_pose_.y - current_pose_.y, 2));
            double velocity = 0.2 * distance;
            double preheading = std::atan2(
                target_pose_.y - current_pose_.y, 
                target_pose_.x - current_pose_.x);
            double orientation = std::atan2(2 * (current_orientation_.w * current_orientation_.z + 
                                                current_orientation_.x * current_orientation_.y), 
                                            1 - 2 * (std::pow(current_orientation_.y, 2) + std::pow(current_orientation_.z, 2)));
            double heading = preheading - orientation;
            double heading_corrected = std::atan2(std::sin(heading), std::cos(heading));
            double angular = std::max(-angle_max, std::min(heading_corrected, angle_max));
            velocity = std::max(-velocity_max, std::min(velocity, velocity_max));
            return {velocity, angular, distance};
        }
        // void readValues() {
        //     std::unique_lock<std::mutex> lock(mtx);
        //     if (numberPtr) {
        //         RCLCPP_INFO(this->get_logger(), "Values read as %d", *numberPtr);
        //     }
        // }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    int five = 0;
    bool inited_waypoints = false;
    geometry_msgs::msg::Pose pose;
    sensor_msgs::msg::NavSatFix gps;
    std::vector<geometry_msgs::msg::PoseStamped> points;

    auto modifier = std::make_shared<Locator>();
    modifier->set_values(&five, &inited_waypoints, &pose, &gps, &points);
    auto reader = std::make_shared<Navigator>();
    reader->set_values(&five, &inited_waypoints, &pose, &gps, &points);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(modifier);
    executor.add_node(reader);
    try {
        executor.spin();
    } catch (...) {
        executor.remove_node(modifier);
        executor.remove_node(reader);
        rclcpp::shutdown();
    }
    return 0;
}