#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode()
  : Node("subscriber_node_manipulator")
  {
    subscriber_ = create_subscription<std_msgs::msg::Int32>(
      "int_topic", 10,
      std::bind(&SubscriberNode::callback, this, _1));
  }

  void callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data % 2 == 0)
    {
        data_even_flag = true;
        data_odd_flag = false;
    }
    else
    {
        data_even_flag = false;
        data_odd_flag = true;   
    }

    RCLCPP_INFO(get_logger(), "Hello %d\nEven: %s\n Odd: %s", msg->data, data_even_flag? "true":"false", data_odd_flag? "true":"false");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
  bool data_even_flag;
  bool data_odd_flag;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SubscriberNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
