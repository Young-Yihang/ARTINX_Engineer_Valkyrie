/**
 * @file mission_executor_node.cpp
 * @brief Application Layer - 任务状态机 + 轨迹管理 TUI v3.0
 *
 * 双视图模式:
 *   [M] 状态机视图 (默认) — 从 mission_sequence.yaml 加载有序状态链
 *       [E] 执行当前状态轨迹 → 成功后推进
 *       [X] 全局复位 → 回 IDLE
 *   [T] 轨迹管理视图 — 列表/执行/保存/删除/详情 (原 v2.0 功能)
 *   [C] 笛卡尔输入   — 调用 /move_to_cartesian_rpy 调试位姿
 */

#include <signal.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <thread>
#include <vector>

#include "arv_v1_interfaces/srv/list_trajectories.hpp"
#include "arv_v1_interfaces/srv/load_trajectory.hpp"
#include "arv_v1_interfaces/srv/move_to_cartesian_rpy.hpp"
#include "arv_v1_interfaces/srv/save_last_trajectory.hpp"

using LoadTrajectory = arv_v1_interfaces::srv::LoadTrajectory;
using ListTrajectories = arv_v1_interfaces::srv::ListTrajectories;
using SaveLastTrajectory = arv_v1_interfaces::srv::SaveLastTrajectory;
using MoveToCartesianRPY = arv_v1_interfaces::srv::MoveToCartesianRPY;
using namespace std::chrono_literals;

// ANSI 颜色
const std::string C_R = "\033[0;31m";
const std::string C_G = "\033[0;32m";
const std::string C_Y = "\033[1;33m";
const std::string C_B = "\033[0;34m";
const std::string C_C = "\033[0;36m";
const std::string C_0 = "\033[0m";
const std::string C_BD = "\033[1m";
const std::string C_DIM = "\033[2m";

// ========== 数据结构 ==========

struct MissionState {
  std::string id;
  std::string trajectory;
  std::string description;
};

struct TrajectoryEntry {
  std::string name;
  std::string description;
};

// ========== 节点 ==========

class MissionExecutorNode : public rclcpp::Node {
public:
  MissionExecutorNode() : Node("mission_executor") {
    trajectory_dir_ =
        std::string(getenv("HOME")) + "/ros2_ws/src/arv_v1_moveit/config/trajectories";
    mission_yaml_path_ =
        std::string(getenv("HOME")) + "/ros2_ws/src/arv_v1_moveit/config/mission_sequence.yaml";

    load_client_ = create_client<LoadTrajectory>("/load_trajectory");
    save_client_ = create_client<SaveLastTrajectory>("/save_last_trajectory");
    cartesian_client_ = create_client<MoveToCartesianRPY>("/move_to_cartesian_rpy");

    RCLCPP_INFO(get_logger(), "Connecting to services...");
    if (!load_client_->wait_for_service(10s)) {
      RCLCPP_ERROR(get_logger(), "load_trajectory service timeout");
      throw std::runtime_error("Connection failed");
    }
    save_client_->wait_for_service(2s);
    cartesian_client_->wait_for_service(2s);

    loadMissionSequence();
    fetchTrajectories();
    RCLCPP_INFO(get_logger(), "Mission Executor v3.0 ready (%zu states, %zu trajectories)",
                states_.size(), trajectories_.size());
  }

  void run() {
    while (rclcpp::ok()) {
      drawUI();
      std::string input = readInput();
      handleInput(input);
      rclcpp::spin_some(get_node_base_interface());
    }
  }

private:
  // ========== 视图模式 ==========
  enum class View { STATE_MACHINE, TRAJECTORY, CARTESIAN };
  View view_ = View::STATE_MACHINE;

  // 输入模式 (轨迹视图复用)
  enum class InputMode {
    COMMAND,
    SAVE_NAME,
    SAVE_DESC,
    DELETE_CONFIRM,
    OVERWRITE_CONFIRM,
    CARTESIAN_INPUT
  };
  InputMode input_mode_ = InputMode::COMMAND;

  // ========== 状态机 ==========
  std::vector<MissionState> states_;
  size_t current_idx_ = 0;
  std::string mission_name_;
  std::string mission_desc_;
  std::string reset_trajectory_;
  std::string mission_yaml_path_;
  bool executing_ = false;

  // ========== 轨迹管理 ==========
  std::vector<TrajectoryEntry> trajectories_;
  std::string trajectory_dir_;
  static constexpr size_t PER_PAGE = 8;
  size_t traj_page_ = 0;

  // ========== 状态栏 ==========
  std::string status_ = "Ready";
  std::mutex status_mu_;

  // ========== 输入缓冲 ==========
  std::string pending_name_;

  // ========== ROS2 客户端 ==========
  rclcpp::Client<LoadTrajectory>::SharedPtr load_client_;
  rclcpp::Client<SaveLastTrajectory>::SharedPtr save_client_;
  rclcpp::Client<MoveToCartesianRPY>::SharedPtr cartesian_client_;

  // ────────────────── 加载 ──────────────────

  void loadMissionSequence() {
    states_.clear();
    if (!std::filesystem::exists(mission_yaml_path_)) {
      RCLCPP_WARN(get_logger(), "Mission YAML not found: %s", mission_yaml_path_.c_str());
      states_.push_back({"IDLE", "", "等待指令"});
      return;
    }
    try {
      auto root = YAML::LoadFile(mission_yaml_path_);
      auto mission = root["mission"];
      mission_name_ = mission["name"].as<std::string>("unnamed");
      mission_desc_ = mission["description"].as<std::string>("");
      reset_trajectory_ = mission["reset_trajectory"].as<std::string>("");
      for (const auto& s : mission["states"]) {
        MissionState ms;
        ms.id = s["id"].as<std::string>("?");
        ms.trajectory = s["trajectory"].as<std::string>("");
        ms.description = s["description"].as<std::string>("");
        states_.push_back(ms);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Parse mission YAML failed: %s", e.what());
      states_.push_back({"IDLE", "", "等待指令(配置错误)"});
    }
    current_idx_ = 0;
  }

  void fetchTrajectories() {
    auto client = create_client<ListTrajectories>("/list_trajectories");
    if (!client->wait_for_service(2s)) return;
    auto req = std::make_shared<ListTrajectories::Request>();
    auto fut = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), fut) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto res = fut.get();
      trajectories_.clear();
      for (size_t i = 0; i < res->names.size(); i++) {
        TrajectoryEntry e;
        e.name = res->names[i];
        e.description = i < res->descriptions.size() ? res->descriptions[i] : "";
        trajectories_.push_back(e);
      }
      traj_page_ = 0;
    }
  }

  // ────────────────── 输入 ──────────────────

  std::string readInput() {
    if (input_mode_ == InputMode::COMMAND) {
      char key;
      std::cin >> key;
      return std::string(1, key);
    }
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::string line;
    std::getline(std::cin, line);
    return line;
  }

  void handleInput(const std::string& input) {
    switch (input_mode_) {
      case InputMode::COMMAND:
        handleCommand(input.empty() ? '\0' : input[0]);
        break;
      case InputMode::SAVE_NAME:
        handleSaveName(input);
        break;
      case InputMode::SAVE_DESC:
        handleSaveDesc(input);
        break;
      case InputMode::DELETE_CONFIRM:
        handleDeleteConfirm(input.empty() ? '\0' : input[0]);
        break;
      case InputMode::OVERWRITE_CONFIRM:
        handleOverwrite(input.empty() ? '\0' : input[0]);
        break;
      case InputMode::CARTESIAN_INPUT:
        handleCartesianInput(input);
        break;
    }
  }

  // ────────────────── 命令分发 ──────────────────

  void handleCommand(char k) {
    // 全局
    if (k == 'q' || k == 'Q') {
      rclcpp::shutdown();
      return;
    }
    if (k == 'h' || k == 'H') {
      showHelp();
      return;
    }

    // 视图切换
    if (k == 'm' || k == 'M') {
      view_ = View::STATE_MACHINE;
      setStatus("切换: 状态机视图");
      return;
    }
    if (k == 't' || k == 'T') {
      view_ = View::TRAJECTORY;
      fetchTrajectories();
      setStatus("切换: 轨迹管理视图");
      return;
    }
    if (k == 'c' || k == 'C') {
      if (!cartesian_client_->service_is_ready()) {
        setStatus("笛卡尔服务未就绪 (cartesian_controller 未启动)");
        return;
      }
      view_ = View::CARTESIAN;
      input_mode_ = InputMode::CARTESIAN_INPUT;
      setStatus("切换: 笛卡尔输入");
      return;
    }

    // 视图内命令
    switch (view_) {
      case View::STATE_MACHINE:
        handleSMCommand(k);
        break;
      case View::TRAJECTORY:
        handleTrajCommand(k);
        break;
      case View::CARTESIAN:
        break;
    }
  }

  // ────────────────── 状态机视图 ──────────────────

  void handleSMCommand(char k) {
    if (k == 'e' || k == 'E') {
      executeCurrentState();
      return;
    }
    if (k == 'x' || k == 'X') {
      resetToIdle();
      return;
    }
    if (k == 'r' || k == 'R') {
      loadMissionSequence();
      setStatus("状态机配置已重新加载");
    }
  }

  void executeCurrentState() {
    if (executing_) {
      setStatus("正在执行, 请等待...");
      return;
    }
    if (current_idx_ >= states_.size()) {
      setStatus("所有状态已完成");
      return;
    }

    auto& st = states_[current_idx_];
    if (st.trajectory.empty()) {
      // 无轨迹状态直接推进
      if (current_idx_ + 1 < states_.size()) {
        current_idx_++;
        setStatus("[" + st.id + "] 跳过(无轨迹) → " + states_[current_idx_].id);
      } else {
        setStatus("流程结束, 按 [X] 复位");
      }
      return;
    }

    executing_ = true;
    setStatus("执行: " + st.id + " → " + st.trajectory + "...");

    auto req = std::make_shared<LoadTrajectory::Request>();
    req->name = st.trajectory;
    req->execute = true;
    auto fut = load_client_->async_send_request(req);

    std::thread([this, fut = std::move(fut), id = st.id]() mutable {
      try {
        auto res = fut.get();
        std::lock_guard<std::mutex> lk(status_mu_);
        if (res->success) {
          status_ = "完成: " + id + " (" + std::to_string(res->duration) + "s)";
          if (current_idx_ + 1 < states_.size()) {
            current_idx_++;
          } else {
            status_ += " — 流程结束, 按 [X] 复位";
          }
        } else {
          status_ = "失败: " + id + " — " + res->message + " (按 [E] 重试)";
        }
      } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lk(status_mu_);
        status_ = "错误: " + std::string(e.what());
      }
      executing_ = false;
    }).detach();
  }

  void resetToIdle() {
    if (executing_) {
      setStatus("正在执行, 无法复位");
      return;
    }

    if (!reset_trajectory_.empty()) {
      executing_ = true;
      setStatus("复位: 执行 " + reset_trajectory_ + "...");
      auto req = std::make_shared<LoadTrajectory::Request>();
      req->name = reset_trajectory_;
      req->execute = true;
      auto fut = load_client_->async_send_request(req);

      std::thread([this, fut = std::move(fut)]() mutable {
        try {
          auto res = fut.get();
          std::lock_guard<std::mutex> lk(status_mu_);
          current_idx_ = 0;
          status_ = res->success ? "已复位到 IDLE" : ("复位失败: " + res->message);
        } catch (const std::exception& e) {
          std::lock_guard<std::mutex> lk(status_mu_);
          current_idx_ = 0;
          status_ = "复位错误: " + std::string(e.what());
        }
        executing_ = false;
      }).detach();
    } else {
      current_idx_ = 0;
      setStatus("已复位到 IDLE");
    }
  }

  // ────────────────── 轨迹管理视图 ──────────────────

  void handleTrajCommand(char k) {
    if (k == 'r' || k == 'R') {
      fetchTrajectories();
      setStatus("列表已刷新");
      return;
    }
    if (k == 's' || k == 'S') {
      input_mode_ = InputMode::SAVE_NAME;
      return;
    }
    if (k == 'd' || k == 'D') {
      if (trajectories_.empty()) {
        setStatus("无轨迹可删除");
        return;
      }
      input_mode_ = InputMode::DELETE_CONFIRM;
      return;
    }
    if (k == 'n' || k == 'N') {
      size_t pages = (trajectories_.size() + PER_PAGE - 1) / PER_PAGE;
      if (pages > 1) traj_page_ = (traj_page_ + 1) % pages;
      return;
    }
    if (k == 'p' || k == 'P') {
      size_t pages = (trajectories_.size() + PER_PAGE - 1) / PER_PAGE;
      if (pages > 1) traj_page_ = (traj_page_ + pages - 1) % pages;
      return;
    }
    if (k >= '1' && k <= '9') {
      size_t idx = (k - '1') + traj_page_ * PER_PAGE;
      if (idx < trajectories_.size()) executeTraj(trajectories_[idx].name);
      return;
    }
  }

  void executeTraj(const std::string& name) {
    if (executing_) {
      setStatus("正在执行, 请等待");
      return;
    }
    executing_ = true;
    setStatus("执行轨迹: " + name + "...");
    auto req = std::make_shared<LoadTrajectory::Request>();
    req->name = name;
    req->execute = true;
    auto fut = load_client_->async_send_request(req);
    std::thread([this, fut = std::move(fut), name]() mutable {
      try {
        auto res = fut.get();
        std::lock_guard<std::mutex> lk(status_mu_);
        status_ = res->success ? ("完成: " + name + " (" + std::to_string(res->duration) + "s)")
                               : ("失败: " + res->message);
      } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lk(status_mu_);
        status_ = "错误: " + std::string(e.what());
      }
      executing_ = false;
    }).detach();
  }

  void handleSaveName(const std::string& name) {
    if (name.empty() || name.find(' ') != std::string::npos) {
      setStatus("名称无效 (不能为空或含空格)");
      input_mode_ = InputMode::COMMAND;
      return;
    }
    bool exists = false;
    for (const auto& t : trajectories_) {
      if (t.name == name) {
        exists = true;
        break;
      }
    }
    pending_name_ = name;
    if (exists) {
      input_mode_ = InputMode::OVERWRITE_CONFIRM;
    } else {
      input_mode_ = InputMode::SAVE_DESC;
    }
  }

  void handleSaveDesc(const std::string& desc) {
    doSave(pending_name_, desc);
    input_mode_ = InputMode::COMMAND;
  }

  void handleOverwrite(char k) {
    if (k == 'y' || k == 'Y') {
      input_mode_ = InputMode::SAVE_DESC;
    } else {
      setStatus("保存取消");
      input_mode_ = InputMode::COMMAND;
    }
  }

  void doSave(const std::string& name, const std::string& desc) {
    if (!save_client_->service_is_ready()) {
      setStatus("保存服务不可用");
      return;
    }
    setStatus("保存: " + name + "...");
    auto req = std::make_shared<SaveLastTrajectory::Request>();
    req->name = name;
    req->description = desc;
    auto fut = save_client_->async_send_request(req);
    std::thread([this, fut = std::move(fut), name]() mutable {
      try {
        auto res = fut.get();
        std::lock_guard<std::mutex> lk(status_mu_);
        status_ = res->success ? ("已保存: " + name) : ("保存失败: " + res->message);
      } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lk(status_mu_);
        status_ = "保存错误: " + std::string(e.what());
      }
    }).detach();
    fetchTrajectories();
  }

  void handleDeleteConfirm(char k) {
    if (k == 'c' || k == 'C') {
      setStatus("删除取消");
      input_mode_ = InputMode::COMMAND;
      return;
    }
    if (k >= '1' && k <= '9') {
      size_t idx = (k - '1') + traj_page_ * PER_PAGE;
      if (idx < trajectories_.size()) {
        std::string path = trajectory_dir_ + "/" + trajectories_[idx].name + ".yaml";
        try {
          if (std::filesystem::exists(path)) {
            std::filesystem::remove(path);
            setStatus("已删除: " + trajectories_[idx].name);
            fetchTrajectories();
          }
        } catch (const std::exception& e) {
          setStatus("删除错误: " + std::string(e.what()));
        }
      }
    }
    input_mode_ = InputMode::COMMAND;
  }

  // ────────────────── 笛卡尔输入 ──────────────────

  void handleCartesianInput(const std::string& line) {
    if (line == "q" || line == "Q" || line == "back") {
      view_ = View::STATE_MACHINE;
      input_mode_ = InputMode::COMMAND;
      setStatus("返回状态机视图");
      return;
    }

    // 格式: x y z roll pitch yaw [vel_scale acc_scale]
    std::istringstream iss(line);
    double x, y, z, roll, pitch, yaw;
    double vel = 0.5, acc = 0.5;
    if (!(iss >> x >> y >> z >> roll >> pitch >> yaw)) {
      setStatus("格式错误, 需要: x y z roll pitch yaw [vel acc]");
      return;
    }
    iss >> vel >> acc;  // 可选

    std::ostringstream label;
    label << std::fixed << std::setprecision(3);
    label << "笛卡尔→ (" << x << ", " << y << ", " << z << ")...";
    setStatus(label.str());

    auto req = std::make_shared<MoveToCartesianRPY::Request>();
    req->x = x;
    req->y = y;
    req->z = z;
    req->roll = roll;
    req->pitch = pitch;
    req->yaw = yaw;
    req->velocity_scaling = vel;
    req->acceleration_scaling = acc;
    req->async_execution = false;

    auto fut = cartesian_client_->async_send_request(req);
    std::thread([this, fut = std::move(fut)]() mutable {
      try {
        auto res = fut.get();
        std::lock_guard<std::mutex> lk(status_mu_);
        std::ostringstream oss;
        if (res->success) {
          oss << "到达 (规划" << std::fixed << std::setprecision(2) << res->planning_time
              << "s, 执行" << res->trajectory_duration << "s)";
        } else {
          oss << "失败: " << res->message;
        }
        status_ = oss.str();
      } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lk(status_mu_);
        status_ = "笛卡尔错误: " + std::string(e.what());
      }
    }).detach();
  }

  // ────────────────── UI 绘制 ──────────────────

  void drawUI() {
    std::cout << "\033[2J\033[H";  // clear + home
    drawHeader();
    switch (view_) {
      case View::STATE_MACHINE:
        drawSMView();
        break;
      case View::TRAJECTORY:
        drawTrajView();
        break;
      case View::CARTESIAN:
        drawCartesianView();
        break;
    }
    drawStatus();
    drawPrompt();
  }

  void drawHeader() {
    std::cout << C_C << C_BD << "╔══════════════════════════════════════════════╗\n"
              << "║         ARV_V1 Mission Control v3.0         ║\n"
              << "╠══════════════════════════════════════════════╣\n"
              << C_0;
    std::cout << "║  ";
    auto tab = [&](const std::string& label, View v) {
      if (view_ == v)
        std::cout << C_BD << "[" << label << "]" << C_0;
      else
        std::cout << C_DIM << " " << label << " " << C_0;
      std::cout << "  ";
    };
    tab("M:状态机", View::STATE_MACHINE);
    tab("T:轨迹", View::TRAJECTORY);
    tab("C:笛卡尔", View::CARTESIAN);
    std::cout << "       ║\n";
    std::cout << C_C << "╚══════════════════════════════════════════════╝\n" << C_0 << "\n";
  }

  void drawSMView() {
    std::cout << C_BD << "  任务: " << C_0 << mission_name_;
    if (!mission_desc_.empty()) std::cout << " — " << mission_desc_;
    std::cout << "\n\n";

    for (size_t i = 0; i < states_.size(); i++) {
      std::string marker, color;
      if (i < current_idx_) {
        marker = "✓";
        color = C_G;
      } else if (i == current_idx_) {
        marker = "→";
        color = C_Y;
      } else {
        marker = " ";
        color = C_DIM;
      }

      std::cout << "  " << color << "[" << marker << "] " << std::setw(16) << std::left
                << states_[i].id;
      if (!states_[i].trajectory.empty())
        std::cout << " (" << states_[i].trajectory << ")";
      else
        std::cout << " (无轨迹)";
      std::cout << "  " << states_[i].description;
      if (i == current_idx_) std::cout << "  ← NOW";
      std::cout << C_0 << "\n";
    }

    std::cout << "\n"
              << C_BD << "  操作: " << C_0 << "[E]执行 [X]复位 [R]重载 [T]轨迹 [C]笛卡尔 "
              << "[H]帮助 [Q]退出\n";
  }

  void drawTrajView() {
    if (trajectories_.empty()) {
      std::cout << C_Y << "  无轨迹, 按 [S] 保存\n" << C_0;
    } else {
      size_t pages = (trajectories_.size() + PER_PAGE - 1) / PER_PAGE;
      size_t s = traj_page_ * PER_PAGE;
      size_t e = std::min(s + PER_PAGE, trajectories_.size());

      std::cout << C_BD << "  轨迹列表 (" << traj_page_ + 1 << "/" << pages << "):\n" << C_0;
      for (size_t i = s; i < e; i++) {
        char key = '1' + static_cast<char>(i - s);
        std::cout << "  [" << C_Y << key << C_0 << "] " << std::setw(20) << std::left
                  << trajectories_[i].name << " : " << trajectories_[i].description << "\n";
      }
    }
    std::cout << "\n"
              << C_BD << "  操作: " << C_0 << "[1-9]执行 [S]保存 [D]删除 [N/P]翻页 "
              << "[R]刷新 [M]状态机 [Q]退出\n";
  }

  void drawCartesianView() {
    std::cout << C_BD << "  笛卡尔目标输入模式\n\n"
              << C_0 << "  格式: " << C_Y << "x y z roll pitch yaw [vel_scale acc_scale]\n"
              << C_0 << "  示例: " << C_DIM << "0.3 0.0 0.5 0.0 1.57 0.0 0.5 0.5\n"
              << C_0 << "  输入 " << C_Y << "q" << C_0 << " 返回\n\n"
              << "  参考系: base_link | 末端: link6_2006roll\n"
              << "  角度: 弧度 | 缩放: 0.0~1.0 (默认0.5)\n";
  }

  void drawStatus() {
    std::cout << "\n  Status: ";
    std::lock_guard<std::mutex> lk(status_mu_);
    if (status_.find("失败") != std::string::npos || status_.find("错误") != std::string::npos)
      std::cout << C_R;
    else if (status_.find("完成") != std::string::npos || status_.find("已") != std::string::npos)
      std::cout << C_G;
    else if (status_.find("执行") != std::string::npos || status_.find("...") != std::string::npos)
      std::cout << C_C;
    std::cout << status_ << C_0 << "\n";
  }

  void drawPrompt() {
    switch (input_mode_) {
      case InputMode::COMMAND:
        std::cout << "\n  > ";
        break;
      case InputMode::SAVE_NAME:
        std::cout << "\n  " << C_Y << "轨迹名称(无空格): " << C_0;
        break;
      case InputMode::SAVE_DESC:
        std::cout << "\n  " << C_Y << "输入描述: " << C_0;
        break;
      case InputMode::DELETE_CONFIRM:
        std::cout << "\n  " << C_R << "选择 [1-9] 或 [C]取消: " << C_0;
        break;
      case InputMode::OVERWRITE_CONFIRM:
        std::cout << "\n  " << C_Y << "'" << pending_name_ << "' 已存在, 覆盖? [Y/N]: " << C_0;
        break;
      case InputMode::CARTESIAN_INPUT:
        std::cout << "\n  笛卡尔> ";
        break;
    }
    std::cout.flush();
  }

  void showHelp() {
    std::cout << "\n"
              << C_C << C_BD << "╔══════════════════════════════════════════════╗\n"
              << "║                  帮助菜单                    ║\n"
              << "╠══════════════════════════════════════════════╣\n"
              << C_0 << "║ " << C_BD << "全局:" << C_0
              << " [M]状态机 [T]轨迹 [C]笛卡尔 [H]帮助 [Q]退出 ║\n"
              << "║ " << C_BD << "状态机:" << C_0
              << " [E]执行→推进 [X]复位 [R]重载yaml          ║\n"
              << "║ " << C_BD << "轨迹:" << C_0 << " [1-9]执行 [S]保存 [D]删除 [N/P]翻页       ║\n"
              << "║ " << C_BD << "笛卡尔:" << C_0 << " x y z rpy [vel acc] / q返回              ║\n"
              << "║ " << C_BD << "工作流:" << C_0
              << "                                           ║\n"
              << "║   笛卡尔找位姿 → RViz规划 → [T][S]保存     ║\n"
              << "║   → 编辑mission_sequence.yaml → [M][E]执行 ║\n"
              << C_C << C_BD << "╚══════════════════════════════════════════════╝\n"
              << C_0 << "\n  " << C_Y << "Enter 继续..." << C_0;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get();
  }

  void setStatus(const std::string& s) {
    std::lock_guard<std::mutex> lk(status_mu_);
    status_ = s;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MissionExecutorNode>();
    node->run();
  } catch (const std::exception& e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
