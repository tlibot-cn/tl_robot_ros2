#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "tl_driver/arm_sdk.hpp"
#include "tl_driver/convert/msg_convert.hpp"
#include "tl_driver/node/registrars.hpp"
#include "tl_driver/tl_driver.h"

#include "tl_ros2_interface/msg/job_file_name.hpp"
#include "tl_ros2_interface/srv/get_all_job_file_name.hpp"
#include "tl_ros2_interface/srv/job_insert_move.hpp"
#include "tl_ros2_interface/srv/job_run.hpp"

namespace tl_driver
{
namespace
{

void handle_get_all_job_filename_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::GetAllJobFileName::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetAllJobFileName::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<std::vector<std::string>> robotsFile;
  tl::Result ret = tl::job_get_all_jobfile_name(node.session().fd(), robotsFile);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Get all job filename successfully" : "Failed to get all job filename";
  response->robots_file.clear();
  for (const auto& file_names : robotsFile)
  {
    tl_ros2_interface::msg::JobFileName msg;
    msg.file_name = file_names;
    response->robots_file.push_back(msg);
  }
}

void handle_job_run_service(DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::JobRun::Request> request,
                            std::shared_ptr<tl_ros2_interface::srv::JobRun::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::job_run(node.session().fd(), request->job_name);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Job run successfully" : "Failed to run job";

  // // 运行完成后会自动下电
  // power_off();
  // power_on();
}

void handle_job_delete_service(DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::JobRun::Request> request,
                               std::shared_ptr<tl_ros2_interface::srv::JobRun::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::job_delete(node.session().fd(), request->job_name);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Job delete successfully" : "Failed to delete job";
}

void handle_job_insert_movej_service(DriverNode& node,
                                     const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
                                     std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::MoveCmd cmd = convert::to_move_cmd(request->cmd);

  tl::Result ret = tl::job_insert_moveJ(node.session().fd(), request->line, cmd);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Job insert movej successfully" : "Failed to insert job movej";
}

void handle_job_insert_movel_service(DriverNode& node,
                                     const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
                                     std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::MoveCmd cmd = convert::to_move_cmd(request->cmd);

  tl::Result ret = tl::job_insert_moveL(node.session().fd(), request->line, cmd);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Job insert movel successfully" : "Failed to insert job movel";
}

void handle_job_insert_imove_service(DriverNode& node,
                                     const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
                                     std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::MoveCmd cmd = convert::to_move_cmd(request->cmd);

  tl::Result ret = tl::job_insert_imove(node.session().fd(), request->line, cmd);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Job insert imove successfully" : "Failed to insert job imove";
}

void handle_job_insert_movec_service(DriverNode& node,
                                     const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
                                     std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::MoveCmd cmd = convert::to_move_cmd(request->cmd);

  tl::Result ret = tl::job_insert_moveC(node.session().fd(), request->line, cmd);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Job insert movec successfully" : "Failed to insert job movec";
}

} // namespace

void register_job_services(DriverNode& node)
{
  node.create_service<tl_ros2_interface::srv::GetAllJobFileName>(
      "/tl_driver/get_all_job_filename",
      std::bind(&handle_get_all_job_filename_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::JobRun>(
      "/tl_driver/job_run",
      std::bind(&handle_job_run_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::JobRun>(
      "/tl_driver/job_delete",
      std::bind(&handle_job_delete_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::JobInsertMove>(
      "/tl_driver/job_insert_moveJ",
      std::bind(&handle_job_insert_movej_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::JobInsertMove>(
      "/tl_driver/job_insert_moveL",
      std::bind(&handle_job_insert_movel_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::JobInsertMove>(
      "/tl_driver/job_insert_imove",
      std::bind(&handle_job_insert_imove_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::JobInsertMove>(
      "/tl_driver/job_insert_moveC",
      std::bind(&handle_job_insert_movec_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
}

} // namespace tl_driver
