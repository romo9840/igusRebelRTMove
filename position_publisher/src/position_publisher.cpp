#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <vector>
#include <sstream>

class PositionPublisher : public rclcpp::Node
{
    public:
        PositionPublisher() : Node("position_publisher"), current_index(0)
        {
            // Publishes angle position commands
            publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("rebel_arm_controller/joint_trajectory", 10);
            
            // Either Publish a fixed list of target positions every N ms 
            // or use a subscriber to listen to new position commands from detected kinect arm movements

            //timer_ = this->create_wall_timer(
            //    std::chrono::milliseconds(1000), std::bind(&PositionPublisher::timer_callback, this));

            // Subscriber to listen to new position commands (from kinect movements)
            subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                "kinect_position_commands", 10, std::bind(&PositionPublisher::positionCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "\n Subscriber node has been initialized\n");
        }

    private:
        void positionCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
            // Triggered when a new position was sent by the kinect arm detection
            RCLCPP_INFO(this->get_logger(), "\nReceived position command message\n");

            // Construct the message format from the received data
            trajectory_msgs::msg::JointTrajectory message;
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = "base_link";
            message.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
            trajectory_msgs::msg::JointTrajectoryPoint point;

            if (!msg->points.empty()) {
                point.positions = msg->points[0].positions;
                point.time_from_start = rclcpp::Duration::from_seconds(1.0);
                message.points.push_back(point);
            }
  
            std::ostringstream oss;
            oss << "Received new kinect arm command:\n";
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

            publisher_->publish(message);
        }

        void timer_callback()
        {
            trajectory_msgs::msg::JointTrajectory message;
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = "base_link";
            message.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = position_commands_[current_index];
            point.time_from_start = rclcpp::Duration::from_seconds(1.0);
            message.points.push_back(point);
  
            publisher_->publish(message);

            current_index = (current_index + 1) % position_commands_.size();
        }

        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
        
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<std::vector<double>> position_commands_ = {
            {-1.5913761899285837, -1.0188160727877884, -0.9291187804318239, 1.4057603359462394, -0.15103927144757162, -0.3332535573637846}, 
            {-1.5913761899285837, -1.0188160727877884, -0.9291187804318239, 1.4057603359462394, -0.15103927144757162, -0.6332535573637846},
            {-1.5913761899285837, -1.0188160727877884, -0.9291187804318239, 1.4057603359462394, -0.15103927144757162, -0.9332535573637846},
        };
        size_t current_index;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionPublisher>());
    rclcpp::shutdown();
    return 0;
}