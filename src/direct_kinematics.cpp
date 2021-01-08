#include <rclcpp/rclcpp.hpp>
#include "chrono"
#include <tf2/LinearMath/Transform.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <math.h>
#include <std_msgs/msg/float64_multi_array.hpp>

using std::placeholders::_1;

class DirectKinematics : public rclcpp::Node
{
    public:
    DirectKinematics() : Node("direct_kineamtics")
    {
        Base.setIdentity();
        Joint1.setIdentity();
        Link1.setIdentity();
        Joint2.setIdentity();
        Link2.setIdentity();

        Link1.setOrigin(tf2::Vector3(300,0,0));
        Link2.setOrigin(tf2::Vector3(200,0,0));

        publisher1_ = this->create_publisher<geometry_msgs::msg::Transform>("Link1",10);
        publisher2_ = this->create_publisher<geometry_msgs::msg::Transform>("Link2",10);

        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("Theta", 10, std::bind(&DirectKinematics::theta_recieved, this, _1));
    }

    private:
    tf2::Transform Base = tf2::Transform();
    tf2::Transform Joint1 = tf2::Transform();
    tf2::Transform Link1 = tf2::Transform();
    tf2::Transform Joint2 = tf2::Transform();
    tf2::Transform Link2 = tf2::Transform();
    double theta[2];
    rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr publisher1_;
    rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr publisher2_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    geometry_msgs::msg::Transform message = geometry_msgs::msg::Transform();

    void theta_recieved(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        this->theta[0] = (M_PI / 180) * (msg->data[0]);
        this->theta[1] = (M_PI / 180) * (msg->data[1]);

        Joint1.setRotation(tf2::Quaternion(tf2::Vector3(0,0,1),theta[0]));
        Joint2.setRotation(tf2::Quaternion(tf2::Vector3(0,0,1),theta[1]));

        tf2::convert(Base*Joint1*Link1,message);
        publisher1_->publish(message);

        tf2::convert(Base*Joint1*Link1*Joint2*Link2,message);
        publisher2_->publish(message);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<DirectKinematics>());
    rclcpp::shutdown();

    return 0;
}