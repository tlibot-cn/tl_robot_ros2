#ifndef TL_TELEOP_F710__TL_TELEOP_F710_SIM_BRIDGE_H_
#define TL_TELEOP_F710__TL_TELEOP_F710_SIM_BRIDGE_H_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ServojSimBridge : public rclcpp::Node
{
public:
  ServojSimBridge();
  ~ServojSimBridge() = default;

private:
  void servojCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  std::string position_controller_topic_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr servoj_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pos_pub_;
};

#endif // TL_TELEOP_F710__TL_TELEOP_F710_SIM_BRIDGE_H_
