#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_srvs/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounterNode: public rclcpp::Node
{
    public:
        NumberCounterNode(): Node("number_counter")
        {
            total = 0;
            publisher = this->create_publisher<std_msgs::msg::Int64>("number_count", 10);

            subscriber = this->create_subscription<std_msgs::msg::Int64>(
                "number",
                10,
                std::bind(&NumberCounterNode::getNumber, this, std::placeholders::_1)
            );

            server = this->create_service<std_srvs::srv::SetBool>(
                "reset_counter",
                std::bind(&NumberCounterNode::resetCounter, this, _1, _2)
            );
            
            RCLCPP_INFO(this->get_logger(), "NumberCounterNode is working");
        }

    private:
        void getNumber(const std_msgs::msg::Int64::SharedPtr msgIn)
        {
            total += msgIn->data;
            auto msgOut = std_msgs::msg::Int64();
            msgOut.data = total;
            publisher->publish(msgOut);
        }

        void resetCounter(
            const std_srvs::srv::SetBool::Request::SharedPtr request,
            const std_srvs::srv::SetBool::Response::SharedPtr response
        ) {
            if (request->data == true) {
                total = 0;
                response->success = true;
                response->message = "";
                return;
            }

            response->success = false;
            response->message = "The counter hasn't be reset because the value passed is 'false'";
        }

        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server;
        int total;
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
