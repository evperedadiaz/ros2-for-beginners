#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RobotNewsStationNode: public rclcpp::Node
{
    public:
        RobotNewsStationNode(): Node("robot_news_station")
        {
            this->declare_parameter("robot_name", "C3P2");

            publisher = this->create_publisher<std_msgs::msg::String>("robot_news", 10);
            robot_name = this->get_parameter("robot_name").as_string();
            counter_ = 0;
            timer = this->create_wall_timer(
                std::chrono::milliseconds(5000),
                std::bind(&RobotNewsStationNode::publishNews, this)
            );
            RCLCPP_INFO(this->get_logger(), "Robot News Station has been started");
        }

    private:
        void publishNews()
        {
            counter_++;
            auto msg = std_msgs::msg::String();
            msg.data = "Hi, this is " + robot_name + " from the Robot News Station - " + std::to_string(counter_);
            publisher->publish(msg);
        }

        std::string robot_name;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr timer;
        int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //auto node = std::make_shared<rclcpp::Node>("cpp_test");
    //RCLCPP_INFO(node->get_logger(), "Hello to my Cpp Node");
    auto node = std::make_shared<RobotNewsStationNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
