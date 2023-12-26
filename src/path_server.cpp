#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

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
        nav_msgs::msg::Path* path;

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

        void set_values(
            int* numPtr,
            bool* inited_waypointsPtr,
            geometry_msgs::msg::Pose* posePtr,
            sensor_msgs::msg::NavSatFix* gpsPtr,
            std::vector<geometry_msgs::msg::PoseStamped>* pointsPtr,
            nav_msgs::msg::Path* pathPtr
        ) {
            RCLCPP_INFO(this->get_logger(), "Setting values");
            numberPtr = numPtr;
            inited_waypoints = inited_waypointsPtr;
            pose = posePtr;
            gps = gpsPtr;
            points = pointsPtr;
            path = pathPtr;
        }

        void change_values() {
            std::lock_guard<std::mutex> lock(mtx);
            if (numberPtr) {
                (*numberPtr) += 1;
                RCLCPP_INFO(this->get_logger(), "Values changed to %d", *numberPtr);
            }
        }
        void sync_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& fix, const nav_msgs::msg::Odometry::ConstSharedPtr& odom) {
            std::lock_guard<std::mutex> lock(mtx);
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
            std::lock_guard<std::mutex> lock(mtx);
            path->header.stamp = this->now();
            path->header.frame_id = "map";
            if (points && inited_waypoints){
                for (auto &i : *points){
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = path->header;
                    pose.pose.position.x = i.pose.position.x;
                    pose.pose.position.y = i.pose.position.y;
                    path->poses.push_back(pose);
                }
            }
            path_pub->publish(*path);
        }
};

class Navigator : public rclcpp::Node {
    private:
        int* numberPtr;
        bool* inited_waypoints;
        geometry_msgs::msg::Pose* pose;
        sensor_msgs::msg::NavSatFix* gps;
        std::vector<geometry_msgs::msg::PoseStamped>* points;
        nav_msgs::msg::Path* path;

        std::mutex mtx;
        rclcpp::TimerBase::SharedPtr timer_;

    public:
        Navigator() : Node("path_server_2"){
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Navigator::readValues, this));
        }

        void set_values(
            int* numPtr,
            bool* inited_waypointsPtr,
            geometry_msgs::msg::Pose* posePtr,
            sensor_msgs::msg::NavSatFix* gpsPtr,
            std::vector<geometry_msgs::msg::PoseStamped>* pointsPtr,
            nav_msgs::msg::Path* pathPtr
        ) {
            RCLCPP_INFO(this->get_logger(), "Setting values");
            numberPtr = numPtr;
            inited_waypoints = inited_waypointsPtr;
            pose = posePtr;
            gps = gpsPtr;
            points = pointsPtr;
            path = pathPtr;
        }

        void readValues() {
            std::lock_guard<std::mutex> lock(mtx);
            if (numberPtr) {
                RCLCPP_INFO(this->get_logger(), "Values read as %d", *numberPtr);
            }
        }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    int five = 0;
    bool inited_waypoints = false;
    geometry_msgs::msg::Pose pose;
    sensor_msgs::msg::NavSatFix gps;
    std::vector<geometry_msgs::msg::PoseStamped> points;
    nav_msgs::msg::Path path;

    auto modifier = std::make_shared<Locator>();
    modifier->set_values(&five, &inited_waypoints, &pose, &gps, &points, &path);
    auto reader = std::make_shared<Navigator>();
    reader->set_values(&five, &inited_waypoints, &pose, &gps, &points, &path);

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


// class GetThePosition: public rclcpp::Node{
//     private:
//         nav_msgs::msg::Path path;
//         bool inited_waypoints;
//         message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub;
//         message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_sub;
//         typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::NavSatFix> MySyncPolicy;
//         message_filters::Synchronizer<MySyncPolicy> sync{MySyncPolicy(10), odom_sub, gps_sub};
//         rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
//         rclcpp::TimerBase::SharedPtr path_timer;
//         geometry_msgs::msg::Pose pose;
//         sensor_msgs::msg::NavSatFix gps;
//         std::vector<geometry_msgs::msg::PoseStamped> points;

//     public:
//         GetThePosition(): Node("get_the_position"){
//             inited_waypoints = false;
//             path_pub = this->create_publisher<nav_msgs::msg::Path>("/fix/waypoints", 10);

//             odom_sub.subscribe(this, "/fix/odom");
//             gps_sub.subscribe(this, "/fix/gps");
//             sync.registerCallback(&GetThePosition::pose_callback, this);
//             path_timer = this->create_wall_timer(2000ms, std::bind(&GetThePosition::path_callback, this));
//         }

//     private:
//         void path_callback(){
//             path.header.stamp = this->now();
//             path.header.frame_id = "map";
//             if (points && !inited_waypoints){
//                 for (auto &i : points){
//                     geometry_msgs::msg::PoseStamped pose;
//                     pose.header = path.header;
//                     pose.pose.position.x = i.pose.position.x;
//                     pose.pose.position.y = i.pose.position.y;
//                     path.poses.push_back(pose);
//                 }
//                 inited_waypoints = true;
//             }
//             path_pub->publish(path);
//         }

//         void pose_callback(const nav_msgs::msg::Odometry::SharedPtr &odom_msg, const sensor_msgs::msg::NavSatFix::SharedPtr &gps_msg){
//             pose.position = odom_msg->pose.pose.position;
//             pose.orientation = odom_msg->pose.pose.orientation;
//             gps = *gps_msg;
//         }
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::NodeOptions options;
//     auto node = std::make_shared<rclcpp::Node>("test", options);
//     rclcpp::executors::SingleThreadedExecutor exec;
//     exec.add_node(node);
//     exec.spin();
//     rclcpp::shutdown();
//     return 0;
// }

