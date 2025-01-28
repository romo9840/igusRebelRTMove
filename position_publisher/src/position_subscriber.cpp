#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class PositionSubscriber : public rclcpp::Node
{
    public:
        PositionSubscriber() : Node("position_subscriber")
        {
            subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "position_commands", 10, std::bind(&PositionSubscriber::topic_callback, this, std::placeholders::_1));
        }

    private:
        void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            position_commands = msg->data;

            std::stringstream ss;
            for (const auto& command : position_commands) {
                ss << command << " ";
            }

            RCLCPP_INFO(this->get_logger(), "position_commands: %s", ss.str().c_str());
        }

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
        std::vector<double> position_commands;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionSubscriber>());
    rclcpp::shutdown();
    return 0;
}