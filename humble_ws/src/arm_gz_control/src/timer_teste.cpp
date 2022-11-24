

#include <memory>
#include <gz/transport/Node.hh>
#include <ignition/msgs.hh>
#include <gz/transport.hh>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      gz_node_ = std::make_shared<gz::transport::Node>();
      gz_node_->Subscribe("/clock", &MinimalSubscriber::topic_callback, this);
      /*subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));*/
    }

  private:
    void topic_callback(const gz::msgs::Clock& msg)
    {
      if (msg.real().sec() > 0){
        RCLCPP_INFO(this->get_logger(), "Startup trigger completed! Shuting down node for launch event handler.");
        rclcpp::shutdown();
      }
      //RCLCPP_INFO(this->get_logger(), "I heard: '%ld'", msg.real().sec());
    }
    //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::shared_ptr<gz::transport::Node> gz_node_;
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

