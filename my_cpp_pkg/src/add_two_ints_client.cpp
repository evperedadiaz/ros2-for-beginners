#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"


class AddTwoIntsClientNode: public rclcpp::Node
{
    public:
        AddTwoIntsClientNode(): Node("add_two_ints_client")
        {
            threads.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 3)));
            threads.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 5, 3)));
            threads.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 3, 3)));
        }

        void callAddTwoIntsService(int a, int b)
        {
            auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
            while (!client->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for server add_two_ints");
            }

            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = a;
            request->b = b;

            auto future = client->async_send_request(request);

            try {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "%d + %d = %ld", a, b, response->sum);
            } catch (const std::exception &e) {
                //std::string error = e.what();
                //RCLCPP_ERROR(this->get_logger(), "Error while calling the service %s", error.c_str());
                RCLCPP_ERROR(this->get_logger(), "Error while calling the service");
            }
        }

    private:
        std::vector<std::thread> threads;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
