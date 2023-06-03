#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

class node_subs_publ_simultaneously : public rclcpp::Node
{
public:
  node_subs_publ_simultaneously()
  : Node("subscriber_publisher_node")
  {
    subscriber_ = create_subscription<std_msgs::msg::Int32>(
      "int_topic", 10,
      std::bind(&node_subs_publ_simultaneously::callback_subscribtion, this, _1));
    
    publisher_ = create_publisher<std_msgs::msg::Int32>("yopic_y", 10);
    timer_ = create_wall_timer(500ms, std::bind(&node_subs_publ_simultaneously::timer_callback, this));

  }
  void callback_subscribtion(const std_msgs::msg::Int32::SharedPtr msg)
  {
    // data_from_topic_x = msg->data;
    RCLCPP_INFO(get_logger(), "Data recieved: %d", msg->data);
    data_from_topic_x.data = msg->data;
  }

  void timer_callback()
  {
    data_to_topic_y.data = 2*data_from_topic_x.data;
    RCLCPP_INFO(get_logger(), "Data sent: %d", data_to_topic_y.data);
    publisher_->publish(data_to_topic_y);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  std_msgs::msg::Int32 data_from_topic_x;
  std_msgs::msg::Int32 data_to_topic_y;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<node_subs_publ_simultaneously>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}