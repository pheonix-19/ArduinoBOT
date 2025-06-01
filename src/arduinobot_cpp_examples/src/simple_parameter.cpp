#include<rclcpp/rclcpp.hpp>
#include<string>
#include<rcl_interfaces/msg/set_parameters_result.hpp>
#include<vector>
#include<memory>

using std::placeholders::_1;

class SimpleParameter : public rclcpp::Node
{
public:
    SimpleParameter() : Node("simple_parameter")
    {
        declare_parameter<int>("simple_int_parameter", 20);
        declare_parameter<std::string>("simple_string_parameter", "Ayush");

        param_callback_handle_ = add_on_set_parameters_callback( std::bind(&SimpleParameter:: paramChangeCallback, this, std::placeholders::_1));
    }
private:
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> &paramter){
        rcl_interfaces::msg::SetParametersResult result;
        
        

        for (const auto &param : paramter) {
            if (param.get_name() == "simple_int_parameter" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                RCLCPP_INFO(this->get_logger(), "Simple int parameter changed to: %ld", param.as_int());
                result.successful = true;
            }
            if (param.get_name() == "simple_string_parameter" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                
                RCLCPP_INFO(this->get_logger(), "Simple string parameter changed to: %s", param.as_string().c_str());
                result.successful = true;
            }
        }
        return result;

    }
     

};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleParameter>();
    rclcpp::spin(node);
    rclcpp::shutdown();


    
    return 0;
}
//   _successful_type successful;
//   using _reason_type =
//     std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;