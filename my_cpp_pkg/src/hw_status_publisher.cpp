#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

class HardwareStatusPublisherNode: public rclcpp::Node
{
    public:
        HardwareStatusPublisherNode(): Node("hardware_status_publisher")
        {
            publisher = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
            timer = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&HardwareStatusPublisherNode::publichHardwareStatus, this)
            );
            RCLCPP_INFO(this->get_logger(), "HardwareStatusPublisherNode has been started");
        }

    private:
        void publichHardwareStatus()
        {
            auto msg = my_robot_interfaces::msg::HardwareStatus();
            msg.temperature = 14;
            msg.are_motors_ready = true;
            msg.degug_message = "C++ publisher is here";
            publisher->publish(msg);
        }

        rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //auto node = std::make_shared<rclcpp::Node>("cpp_test");
    //RCLCPP_INFO(node->get_logger(), "Hello to my Cpp Node");
    auto node = std::make_shared<HardwareStatusPublisherNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
