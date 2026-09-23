#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/empty.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "bunker_msgs/msg/bunker_status.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class OdometerNode : public rclcpp::Node
{
public:
    OdometerNode() : Node("odometer")
    {
        // Subscriber
        subscription_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>(
            "/bunker_status", 10,
            std::bind(&OdometerNode::callback, this, _1));

        // Publisher
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/project_odom", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Reset service
        srv_ = this->create_service<std_srvs::srv::Empty>(
            "reset",
            std::bind(&OdometerNode::resetCallback, this, _1, _2));

        // Initialize variables
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        k = 0.001169;
        L = 0.713317;

        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Odometer node started");
    }

private:
    void callback(const bunker_msgs::msg::BunkerStatus::SharedPtr msg)
    {
       
        double vR = k*msg->actuator_states[0].rpm;   
        double vL = k*msg->actuator_states[1].rpm;   
        double v = (vR+vL)/2; 
        double omega = (vR-vL)/L;

        
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        
        theta_ += omega * dt;
        x_ += v * cos(theta_+(dt*omega/2)) * dt;
        y_ += v * sin(theta_+(dt*omega/2)) * dt;
        

        // Quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        // Publish Odometry
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link2";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = omega;

        publisher_->publish(odom);

        // Publish TF
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link2";

        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }

    void resetCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "Odometry reset!");
    }

    // Variables
    double x_, y_, theta_;
    double k, L;
    rclcpp::Time last_time_;

    // ROS interfaces
    rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_; 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometerNode>());
    rclcpp::shutdown();
    return 0;
}

