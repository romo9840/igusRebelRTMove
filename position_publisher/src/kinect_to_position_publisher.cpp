#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <vector>
#include <sstream>

class KinectPositionPublisher : public rclcpp::Node
{
    public:
        KinectPositionPublisher() : Node("kinect_to_position_publisher"), current_index(0)
        {
            // Publishes angle position commands
            publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("kinect_position", 10);
            
            // Currently Publish a fixed list of target positions every N ms
            // Later replaced by the kinect arm detection
            timer_ = this->create_wall_timer(
            std::chrono::milliseconds(3000), std::bind(&KinectPositionPublisher::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            RCLCPP_INFO(this->get_logger(), "\n Timer callback\n");

            trajectory_msgs::msg::JointTrajectory message;
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = "base_link";
            message.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = position_commands_[current_index];
            point.time_from_start = rclcpp::Duration::from_seconds(1.0);
            message.points.push_back(point);
  
            publisher_->publish(message);

            std::ostringstream oss;
            oss << "Published:\n";
            oss << "Header:\n";
            oss << "  stamp: " << message.header.stamp.sec << "s " << message.header.stamp.nanosec << "ns\n";
            oss << "  frame_id: " << message.header.frame_id << "\n";
            oss << "Joint Names: ";
            for (const auto &name : message.joint_names) {
                oss << name << " ";
            }
            oss << "\nPoints:\n";
            for (const auto &pos : point.positions) {
                oss << "  position: " << pos << "\n";
            }

            RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

            current_index = (current_index + 1) % position_commands_.size();
        }

        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<std::vector<double>> position_commands_ = {
            {-1.5913761899285837, -1.0188160727877884, -0.9291187804318239, 1.4057603359462394, -0.15103927144757162, -0.0}, 
            {-1.5913761899285837, -1.0188160727877884, -0.9291187804318239, 1.4057603359462394, -0.15103927144757162, -0.3}, 
            {-1.5913761899285837, -1.0188160727877884, -0.9291187804318239, 1.4057603359462394, -0.15103927144757162, -0.6},
            {-1.5913761899285837, -1.0188160727877884, -0.9291187804318239, 1.4057603359462394, -0.15103927144757162, -0.9},
            {-1.5913761899285837, -1.0188160727877884, -0.9291187804318239, 1.4057603359462394, -0.15103927144757162, -1.2},
            {-1.5913761899285837, -1.0188160727877884, -0.9291187804318239, 1.4057603359462394, -0.15103927144757162, -1.5},
            {-1.5913761899285837, -1.0188160727877884, -0.9291187804318239, 1.4057603359462394, -0.15103927144757162, -1.2},
            {-1.5913761899285837, -1.0188160727877884, -0.9291187804318239, 1.4057603359462394, -0.15103927144757162, -0.9},
            {-1.5913761899285837, -1.0188160727877884, -0.9291187804318239, 1.4057603359462394, -0.15103927144757162, -0.6},
            {-1.5913761899285837, -1.0188160727877884, -0.9291187804318239, 1.4057603359462394, -0.15103927144757162, -0.3},


        };
        size_t current_index;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinectPositionPublisher>());
    rclcpp::shutdown();
    return 0;
}