#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/string.hpp>
#include<chrono>

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node{

    public:
        SimplePublisher() :Node("simple_publisher"), counter_(0)
        {
            pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
            timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));
        }
        void timerCallback()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello, world! " + std::to_string(counter_++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            pub_->publish(message);
        }
    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        unsigned int counter_;
    };

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePublisher>();;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

