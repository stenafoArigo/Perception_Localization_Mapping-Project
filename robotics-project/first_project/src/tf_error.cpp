#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

#include "first_project/msg/tf_error_msg.hpp"

using namespace std::chrono_literals;

class TfError : public rclcpp::Node
{
public:
    TfError() : Node("tf_error")
    {
        publisher_ = this->create_publisher<first_project::msg::TfErrorMsg>(
            "/tf_error_msg", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(100ms, std::bind(&TfError::on_timer, this));

        travelled_distance_ = 0.0;
        first_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "TF Error node started");
    }

private:
    void on_timer()
    {
        geometry_msgs::msg::TransformStamped t;

        try {
            t = tf_buffer_->lookupTransform("base_link", "base_link2", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Lookup failed: %s", ex.what());
            return;
        }

        float x = t.transform.translation.x;
        float y = t.transform.translation.y;
        float z = t.transform.translation.z;

        float tf_error = std::sqrt(x*x + y*y + z*z);

        travelled_distance_ += tf_error;

        auto elapsed = this->now() - first_time_;

        first_project::msg::TfErrorMsg msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "tf_error";
        msg.tf_error = tf_error;
        msg.time_from_start = elapsed.seconds();  // convert to seconds
        msg.travelled_distance = travelled_distance_;

        publisher_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<first_project::msg::TfErrorMsg>::SharedPtr publisher_;

    float travelled_distance_;
    rclcpp::Time first_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfError>());
    rclcpp::shutdown();
    return 0;
}