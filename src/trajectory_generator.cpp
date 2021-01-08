#include <rclcpp/rclcpp.hpp>
#include "iostream"
#include <std_msgs/msg/float64_multi_array.hpp>
#include "chrono"

using std::placeholders::_1;

enum TrajectoryType
{
    PTP,
    LIN,
    CIRC,
    BEZ
};

class TrajectoryGenerator : public rclcpp::Node
{
    public:
    TrajectoryGenerator() : Node("trajectory_generator")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("Command", 10, std::bind(&TrajectoryGenerator::multi_axis_trajectory_generator,this,_1));
        message.data.resize(2);
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("Theta",10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&TrajectoryGenerator::timer_callback, this));
    }

    private:
    std::vector<double> theta = {0.0 , 0.0};
    std::vector<double> desired_theta = {0.0 , 0.0};
    std_msgs::msg::Float64MultiArray message = std_msgs::msg::Float64MultiArray();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;

    void timer_callback()
    {
        for (int i = 0; i < 2; i++)
        {
            theta[i] += 0.01 * (desired_theta[i] - theta[i]);
        }
        
        message.data = theta;
        publisher_->publish(message);
    }

    void multi_axis_trajectory_generator(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        desired_theta = msg->data;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TrajectoryGenerator>());
    rclcpp::shutdown();

    return 0;

}