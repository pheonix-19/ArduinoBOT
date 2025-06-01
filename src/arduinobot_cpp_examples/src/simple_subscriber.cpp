#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/string.hpp>

class SimpleSubscriber : public rclcpp::Node{
    public:
        SimpleSubscriber(): Node("simple_subscriber"){
            sub_ = create_subscription<std_msgs::msg::String>(
                "chatter", 10, std::bind(&SimpleSubscriber::callback, this, std::placeholders::_1));

        }
        void callback(const std_msgs::msg::String::SharedPtr msg) const {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        }
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;    
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSubscriber>());
    rclcpp::shutdown();
    return 0;
}
// }
// This code is a simple ROS 2 subscriber that listens to a topic named "chatter"
// and prints the received messages to the console. It uses the std_msgs/msg/String message type.