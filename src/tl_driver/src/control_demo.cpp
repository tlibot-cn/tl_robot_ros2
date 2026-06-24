#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tl_ros2_interface/srv/coord_transform.hpp>
#include <tl_ros2_interface/srv/open_servo_j.hpp>
#include <vector>
#include <memory>
#include <cmath>
#include <iomanip>

class ControlNode : public rclcpp::Node
{
public:
  ControlNode() : Node("servoj_control_demo")
  {
    // 创建客户端
    open_servoj_client_ = this->create_client<tl_ros2_interface::srv::OpenServoJ>("/tl_driver/open_servoj");
    close_servoj_client_ = this->create_client<std_srvs::srv::Trigger>("/tl_driver/close_servoj");
    coord_transform_client_ = this->create_client<tl_ros2_interface::srv::CoordTransform>("/tl_driver/coord_transform");

    // 创建发布者 - 下发关节角度到servoJ透传模式
    servoj_pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tl_driver/set_servoj_pos", 10);

    // 创建订阅者 - 订阅目标笛卡尔位姿
    target_pose_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_pos_cartesian", 10, std::bind(&ControlNode::target_pose_callback, this, std::placeholders::_1));

    // 初始化时开启servoJ
    open_servoj();

    RCLCPP_INFO(this->get_logger(), "Cartesian to Joint node initialized");
    RCLCPP_INFO(this->get_logger(), "Input topic: /target_pos_cartesian (Float64MultiArray)");
    RCLCPP_INFO(this->get_logger(), "Output topic: /tl_driver/set_servoj_pos (Float64MultiArray)");
  }

  ~ControlNode()
  {
    // 节点关闭时关闭servoJ
    close_servoj();
  }

private:
  void open_servoj()
  {
    RCLCPP_INFO(this->get_logger(), "Opening servoJ...");

    // 等待服务可用
    if (!open_servoj_client_->wait_for_service(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(this->get_logger(), "/tl_driver/open_servoj service not available");
      return;
    }

    // 构建请求，设置速度、加速度、加加速度限制
    auto request = std::make_shared<tl_ros2_interface::srv::OpenServoJ::Request>();

    // 设置各轴的最大速度 (单位: 度/秒)
    request->vmax = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0};

    // 设置各轴的最大加速度 (单位: 度/秒²)
    request->amax = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0};

    // 设置各轴的最大加加速度 (单位: 度/秒³)
    request->jmax = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0};

    // 异步调用服务
    auto result = open_servoj_client_->async_send_request(
        request, std::bind(&ControlNode::open_servoj_callback, this, std::placeholders::_1));
  }

  void open_servoj_callback(rclcpp::Client<tl_ros2_interface::srv::OpenServoJ>::SharedFuture future)
  {
    auto response = future.get();

    if (response->success)
    {
      RCLCPP_INFO(this->get_logger(), "ServoJ opened successfully: %s", response->message.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open servoJ: %s", response->message.c_str());
    }
  }

  void close_servoj()
  {
    RCLCPP_INFO(this->get_logger(), "Closing servoJ...");

    if (!close_servoj_client_->wait_for_service(std::chrono::seconds(3)))
    {
      RCLCPP_ERROR(this->get_logger(), "/tl_driver/close_servoj service not available");
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = close_servoj_client_->async_send_request(request);

    // 等待响应（同步方式，因为是在析构函数中）
    auto future = result.wait_for(std::chrono::seconds(2));
    if (future == std::future_status::ready)
    {
      auto response = result.get();
      if (response->success)
      {
        RCLCPP_INFO(this->get_logger(), "ServoJ closed successfully: %s", response->message.c_str());
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to close servoJ: %s", response->message.c_str());
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Close servoJ request timeout");
    }
  }

  void target_pose_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // 检查输入数据长度
    if (msg->data.size() < 6)
    {
      RCLCPP_WARN(this->get_logger(), "Invalid input data size: %zu, expected at least 6 values [x, y, z, rx, ry, rz]",
                  msg->data.size());
      return;
    }

    // 提取笛卡尔位姿 [x, y, z, rx, ry, rz]
    std::vector<double> cartesian_pose = {
        msg->data[0], // x
        msg->data[1], // y
        msg->data[2], // z
        msg->data[3], // rx (roll, 度)
        msg->data[4], // ry (pitch, 度)
        msg->data[5]  // rz (yaw, 度)
    };

    RCLCPP_INFO(this->get_logger(), "Received target pose: x=%.3f, y=%.3f, z=%.3f, rx=%.2f, ry=%.2f, rz=%.2f",
                cartesian_pose[0], cartesian_pose[1], cartesian_pose[2], cartesian_pose[3], cartesian_pose[4],
                cartesian_pose[5]);

    // 检查服务是否可用
    if (!coord_transform_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "/tl_driver/coord_transform service not available");
      return;
    }

    // 构建服务请求
    auto request = std::make_shared<tl_ros2_interface::srv::CoordTransform::Request>();
    request->origin_coord = 0;            // 原始坐标系：笛卡尔坐标系
    request->target_coord = 1;            // 目标坐标系：关节坐标系
    request->origin_pos = cartesian_pose; // 输入的笛卡尔位姿
    request->form = 1;                    // 转换形式：完整位姿转换
    request->reference_pos = {};          // 参考位姿（可选）

    // 异步调用服务
    auto result = coord_transform_client_->async_send_request(
        request, std::bind(&ControlNode::transform_callback, this, std::placeholders::_1));
  }

  void transform_callback(rclcpp::Client<tl_ros2_interface::srv::CoordTransform>::SharedFuture future)
  {
    auto response = future.get();

    if (!response->success)
    {
      RCLCPP_ERROR(this->get_logger(), "Coordinate transform failed: %s", response->message.c_str());
      return;
    }

    // 获取转换后的关节角度（单位：度）
    std::vector<double> joint_angles_deg = response->target_pos;

    if (joint_angles_deg.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Transform returned empty result");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Transform success, got %zu joint angles (degrees)", joint_angles_deg.size());

    // 打印关节角度信息
    std::stringstream ss;
    ss << "Joint angles: [";
    for (size_t i = 0; i < joint_angles_deg.size(); ++i)
    {
      ss << std::fixed << std::setprecision(2) << joint_angles_deg[i];
      if (i < joint_angles_deg.size() - 1)
        ss << ", ";
    }
    ss << "] deg";
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    // 直接下发关节角度
    publish_joint_angles(joint_angles_deg);
  }

  void publish_joint_angles(const std::vector<double>& joint_angles)
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = joint_angles;
    servoj_pos_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Published joint angles to /tl_driver/set_servoj_pos");
  }

  // 成员变量
  rclcpp::Client<tl_ros2_interface::srv::OpenServoJ>::SharedPtr open_servoj_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr close_servoj_client_;
  rclcpp::Client<tl_ros2_interface::srv::CoordTransform>::SharedPtr coord_transform_client_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servoj_pos_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pose_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}