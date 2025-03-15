#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class NumberPublisherNode: public rclcpp::Node
{
    public:
        NumberPublisherNode(): Node("number_publisher")
        {
            this->declare_parameter("number_to_publish", 333);
            this->declare_parameter("frequency", 1.0);

            number = this->get_parameter("number_to_publish").as_int();
            double frequency = this->get_parameter("frequency").as_double();
            publisher = this->create_publisher<std_msgs::msg::Int64>("number", 10);
            timer = this->create_wall_timer(
                std::chrono::milliseconds((int) (1000 / frequency)),
                std::bind(&NumberPublisherNode::publishNumber, this)
            );
            RCLCPP_INFO(this->get_logger(), "NumberPublisherNode has been started");
        }

    private:
        void publishNumber()
        {
            auto msg = std_msgs::msg::Int64();
            msg.data = number;
            publisher->publish(msg);
        }

        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr timer;
        int number = 2;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //auto node = std::make_shared<rclcpp::Node>("cpp_test");
    //RCLCPP_INFO(node->get_logger(), "Hello to my Cpp Node");
    auto node = std::make_shared<NumberPublisherNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
