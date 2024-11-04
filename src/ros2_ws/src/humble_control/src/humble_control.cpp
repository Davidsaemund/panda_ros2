#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tutorial_interfaces/srv/set_int.hpp"

using SetInt = tutorial_interfaces::srv::SetInt;

class HumbleControlNode : public rclcpp::Node
{
public:
    HumbleControlNode() : Node("humble_control_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("classify_results", 10);
        service_ = this->create_service<SetInt>("set_classify_results", std::bind(&HumbleControlNode::handle_set_classify_results, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void handle_set_classify_results(const std::shared_ptr<SetInt::Request> request, std::shared_ptr<SetInt::Response> response)
    {
        auto message = std_msgs::msg::Int32();
        message.data = request->data;
        publisher_->publish(message);

        response->success = true;
        response->message = "Classify result set successfully.";
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Service<SetInt>::SharedPtr service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumbleControlNode>());
    rclcpp::shutdown();
    return 0;
}
