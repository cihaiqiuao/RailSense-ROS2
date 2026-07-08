#include <chrono>
#include <csignal>
#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trainros_interfaces/action/record.hpp"

using namespace std::chrono_literals;

class RecorderNode : public rclcpp::Node
{
public:
  using Record = trainros_interfaces::action::Record;
  using GoalHandleRecord = rclcpp_action::ServerGoalHandle<Record>;

  RecorderNode() : Node("trainros_recorder")
  {
    default_output_uri_ = declare_parameter<std::string>("default_output_uri", "data/bags/trainros");
    record_topics_ = declare_parameter<std::vector<std::string>>(
      "record_topics",
      {
        "/imu/data",
        "/gps/fix",
        "/laser/scan",
        "/coupler_detection",
        "/train_state",
        "/stability",
        "/system_status",
        "/diagnostics",
        "/log_status",
      });
    record_camera_ = declare_parameter<bool>("record_camera", false);

    action_server_ = rclcpp_action::create_server<Record>(
      this,
      "record",
      std::bind(&RecorderNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RecorderNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&RecorderNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Recorder Action 已启动，使用 ros2 bag record 执行录制");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Record::Goal> goal)
  {
    if (!goal->record) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lock(process_mutex_);
    if (active_pid_ > 0) {
      RCLCPP_WARN(get_logger(), "已有 rosbag2 录制进程，拒绝新的录制请求");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleRecord>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleRecord> goal_handle)
  {
    std::thread{std::bind(&RecorderNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  pid_t start_rosbag_process(const std::string & output_uri)
  {
    std::vector<std::string> args = {"ros2", "bag", "record", "-o", output_uri};
    std::vector<std::string> topics = record_topics_;
    if (record_camera_ &&
      std::find(topics.begin(), topics.end(), "/camera/image_raw") == topics.end())
    {
      topics.push_back("/camera/image_raw");
    }
    args.insert(args.end(), topics.begin(), topics.end());

    const pid_t pid = ::fork();
    if (pid < 0) {
      return -1;
    }

    if (pid == 0) {
      ::setpgid(0, 0);
      std::vector<char *> argv;
      argv.reserve(args.size() + 1);
      for (auto & arg : args) {
        argv.push_back(arg.data());
      }
      argv.push_back(nullptr);
      ::execvp("ros2", argv.data());
      ::_exit(127);
    }

    ::setpgid(pid, pid);
    return pid;
  }

  void stop_rosbag_process(const pid_t pid)
  {
    if (pid <= 0) {
      return;
    }
    ::kill(-pid, SIGINT);
  }

  bool process_exited(const pid_t pid, int & status)
  {
    const pid_t result = ::waitpid(pid, &status, WNOHANG);
    return result == pid;
  }

  void clear_active_pid(const pid_t pid)
  {
    std::lock_guard<std::mutex> lock(process_mutex_);
    if (active_pid_ == pid) {
      active_pid_ = -1;
    }
  }

  void execute(const std::shared_ptr<GoalHandleRecord> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const std::string output_uri = goal->output_uri.empty() ? default_output_uri_ : goal->output_uri;

    auto feedback = std::make_shared<Record::Feedback>();
    auto result = std::make_shared<Record::Result>();

    const pid_t pid = start_rosbag_process(output_uri);
    if (pid <= 0) {
      result->success = false;
      result->message = "启动 ros2 bag record 失败";
      goal_handle->abort(result);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(process_mutex_);
      active_pid_ = pid;
    }

    RCLCPP_INFO(get_logger(), "rosbag2 录制已启动，pid=%d，输出目录：%s", pid, output_uri.c_str());
    const auto start_time = std::chrono::steady_clock::now();
    int process_status = 0;

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        stop_rosbag_process(pid);
        while (!process_exited(pid, process_status)) {
          std::this_thread::sleep_for(100ms);
        }
        clear_active_pid(pid);
        result->success = true;
        result->message = "录制已取消，rosbag2 已停止";
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "rosbag2 录制已取消，输出目录：%s", output_uri.c_str());
        return;
      }

      if (process_exited(pid, process_status)) {
        clear_active_pid(pid);
        const bool ok = WIFEXITED(process_status) && WEXITSTATUS(process_status) == 0;
        result->success = ok;
        result->message = ok ? "rosbag2 录制正常结束" : "rosbag2 录制进程异常退出";
        if (ok) {
          goal_handle->succeed(result);
        } else {
          goal_handle->abort(result);
        }
        return;
      }

      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time);
      feedback->current_duration_sec = static_cast<float>(elapsed.count()) / 1000.0F;
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(500ms);
    }

    stop_rosbag_process(pid);
    clear_active_pid(pid);
    result->success = false;
    result->message = "ROS 正在关闭，录制已停止";
    goal_handle->abort(result);
  }

  std::string default_output_uri_;
  std::vector<std::string> record_topics_;
  bool record_camera_ = false;
  std::mutex process_mutex_;
  pid_t active_pid_ = -1;
  rclcpp_action::Server<Record>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RecorderNode>());
  rclcpp::shutdown();
  return 0;
}
