#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class NumberCounterNode: public rclcpp::Node
{
    public:
        NumberCounterNode(): Node("smartphone")
        {
            subscriber = this->create_subscription<std_msgs::msg::String>(
                "robot_news",
                10,
                std::bind(&NumberCounterNode::callbackRobotNews, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "Smartphone is working");
        }

    private:
        void callbackRobotNews(const std_msgs::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
        }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //auto node = std::make_shared<rclcpp::Node>("cpp_test");
    //RCLCPP_INFO(node->get_logger(), "Hello to my Cpp Node");
    auto node = std::make_shared<NumberCounterNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
