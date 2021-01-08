#include <rclcpp/rclcpp.hpp>
#include "iostream"
#include "chrono"
#include "string"
#include <std_msgs/msg/string.hpp>
#include "string_view"
#include <std_msgs/msg/float64_multi_array.hpp>

using std::placeholders::_1;

class PrograpmParser : public rclcpp::Node
{
    public:
    PrograpmParser() : Node("program_parser")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>("Program",10,std::bind(&PrograpmParser::program_recieved, this, _1));

        message.data.resize(2);
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("Command",10);
    }

    private:
    std::string full_text;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    std_msgs::msg::Float64MultiArray message = std_msgs::msg::Float64MultiArray();
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

    
    void program_recieved(const std_msgs::msg::String::SharedPtr msg)
    {
        //TODO: need more expandation
        full_text = msg->data;
        std::string::size_type sz;

        message.data[0] = std::stod(full_text, &sz);
        message.data[1] = std::stod(full_text.substr(sz));

        publisher_->publish(message);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<PrograpmParser>());
    rclcpp::shutdown();

    return 0;
}