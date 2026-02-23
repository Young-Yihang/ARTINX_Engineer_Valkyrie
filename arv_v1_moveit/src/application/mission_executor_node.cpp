/**
 * @file mission_executor_node.cpp
 * @brief Application Layer - ncurses TUI for Mission Control (v4.0)
 *
 * Provides a non-blocking, hotkey-driven interface using ncurses.
 * Features strict "Takeover Mode" isolation to prevent hotkey conflicts.
 */

#include <ncurses.h>
#include <signal.h>
#include <yaml-cpp/yaml.h>
#include <clocale>

#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <thread>
#include <vector>

#include "arv_v1_interfaces/srv/gripper_control.hpp"
#include "arv_v1_interfaces/srv/list_trajectories.hpp"
#include "arv_v1_interfaces/srv/load_trajectory.hpp"
#include "arv_v1_interfaces/srv/move_to_cartesian_rpy.hpp"
#include "arv_v1_interfaces/srv/save_last_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"  // For cartesian jogging relative offsets

using LoadTrajectory = arv_v1_interfaces::srv::LoadTrajectory;
using ListTrajectories = arv_v1_interfaces::srv::ListTrajectories;
using SaveLastTrajectory = arv_v1_interfaces::srv::SaveLastTrajectory;
using MoveToCartesianRPY = arv_v1_interfaces::srv::MoveToCartesianRPY;
using GripperControl = arv_v1_interfaces::srv::GripperControl;
using namespace std::chrono_literals;

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

// 日志环形缓冲区
class LogBuffer {
public:
  void add(const std::string& msg, int color_pair) {
    std::lock_guard<std::mutex> lock(mu_);
    if (logs_.size() >= max_size_) logs_.pop_front();
    logs_.push_back({msg, color_pair});
  }
  std::vector<std::pair<std::string, int>> get() {
    std::lock_guard<std::mutex> lock(mu_);
    return {logs_.begin(), logs_.end()};
  }

private:
  std::deque<std::pair<std::string, int>> logs_;
  const size_t max_size_ = 5;  // 底部显示5行日志
  std::mutex mu_;
};

// ========== 颜色定义 ==========
#define COLOR_PAIR_DEFAULT 1
#define COLOR_PAIR_HEADER 2
#define COLOR_PAIR_SUCCESS 3
#define COLOR_PAIR_ERROR 4
#define COLOR_PAIR_WARNING 5
#define COLOR_PAIR_HIGHLIGHT 6

// ========== 节点 ==========

class MissionExecutorNode : public rclcpp::Node {
public:
  MissionExecutorNode() : Node("mission_executor") {
    initNcurses();

    trajectory_dir_ =
        std::string(getenv("HOME")) + "/ros2_ws/src/arv_v1_moveit/config/trajectories";
    mission_yaml_path_ =
        std::string(getenv("HOME")) + "/ros2_ws/src/arv_v1_moveit/config/mission_sequence.yaml";

    load_client_ = create_client<LoadTrajectory>("/load_trajectory");
    save_client_ = create_client<SaveLastTrajectory>("/save_last_trajectory");
    cartesian_client_ = create_client<MoveToCartesianRPY>("/move_to_cartesian_rpy");
    gripper_client_ = create_client<GripperControl>("/gripper_control");

    log("Connecting to services...", COLOR_PAIR_DEFAULT);
    if (!load_client_->wait_for_service(10s)) {
      log("load_trajectory service timeout", COLOR_PAIR_ERROR);
    }
    save_client_->wait_for_service(2s);
    cartesian_client_->wait_for_service(2s);

    loadMissionSequence();
    fetchTrajectories();
    log("Mission Executor v4.0 ready", COLOR_PAIR_SUCCESS);
  }

  ~MissionExecutorNode() { shutdownNcurses(); }

  void run() {
    int ch;
    while (rclcpp::ok() && running_) {
      drawUI();
      ch = getch();  // 非阻塞, timeout=100ms
      if (ch != ERR) {
        handleInput(ch);
      }
      rclcpp::spin_some(get_node_base_interface());
    }
  }

private:
  bool running_ = true;
  LogBuffer log_buffer_;

  // ========== UI/UX 核心状态 ==========
  enum class View { STATE_MACHINE, TRAJECTORY, CARTESIAN, GRIPPER, HELP };
  View view_ = View::STATE_MACHINE;

  bool takeover_mode_ = false;  // "接管模式"，隔离全局按键

  // 弹窗输入状态
  enum class InputMode { NONE, SAVE_NAME, SAVE_DESC, DELETE_CONFIRM, OVERWRITE_CONFIRM };
  InputMode input_mode_ = InputMode::NONE;
  std::string input_buffer_;
  std::string pending_name_;

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
  static constexpr size_t PER_PAGE = 7;
  size_t traj_page_ = 0;

  // ========== 夹爪状态 ==========
  double gripper_torque_cmd_ = 0.0;  // 本地预设想发的力矩
  double gripper_last_sent_ = 0.0;   // 实际最后发送的力矩

  // ========== 笛卡尔状态 ==========
  double cartesian_step_ = 0.05;  // 5cm

  // ========== ROS2 客户端 ==========
  rclcpp::Client<LoadTrajectory>::SharedPtr load_client_;
  rclcpp::Client<SaveLastTrajectory>::SharedPtr save_client_;
  rclcpp::Client<MoveToCartesianRPY>::SharedPtr cartesian_client_;
  rclcpp::Client<GripperControl>::SharedPtr gripper_client_;

  // ────────────────── Ncurses 初始化 ──────────────────

  void initNcurses() {
    initscr();             // 初始化
    cbreak();              // 禁用行缓冲，直接读字符
    noecho();              // 不回显
    keypad(stdscr, TRUE);  // 捕获特殊按键(方向键、F1等)
    timeout(100);          // getch() 阻塞 100ms，保证 10Hz 顺滑刷新

    if (has_colors()) {
      start_color();
      init_pair(COLOR_PAIR_DEFAULT, COLOR_WHITE, COLOR_BLACK);
      init_pair(COLOR_PAIR_HEADER, COLOR_CYAN, COLOR_BLACK);
      init_pair(COLOR_PAIR_SUCCESS, COLOR_GREEN, COLOR_BLACK);
      init_pair(COLOR_PAIR_ERROR, COLOR_RED, COLOR_BLACK);
      init_pair(COLOR_PAIR_WARNING, COLOR_YELLOW, COLOR_BLACK);
      init_pair(COLOR_PAIR_HIGHLIGHT, COLOR_BLACK, COLOR_WHITE);  // 反色
    }
  }

  void shutdownNcurses() { endwin(); }

  void log(const std::string& msg, int color = COLOR_PAIR_DEFAULT) {
    log_buffer_.add(msg, color);
    // 移除 RCLCPP_INFO，防止底层的标准输出与 ncurses 争夺屏幕控制权导致 UI 撕裂/上移
  }

  void logErr(const std::string& msg) { log(msg, COLOR_PAIR_ERROR); }
  void logOk(const std::string& msg) { log(msg, COLOR_PAIR_SUCCESS); }
  void logWarn(const std::string& msg) { log(msg, COLOR_PAIR_WARNING); }

  // ────────────────── 后台加载 ──────────────────

  void loadMissionSequence() {
    std::lock_guard<std::mutex> lk(status_mu_);
    states_.clear();
    if (!std::filesystem::exists(mission_yaml_path_)) {
      logWarn("Mission YAML not found, using IDLE.");
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
      logErr("Parse mission YAML failed.");
      states_.push_back({"IDLE", "", "等待指令(配置错误)"});
    }
    current_idx_ = 0;
  }

  void fetchTrajectories() {
    if (!load_client_->service_is_ready()) return;
    auto client = create_client<ListTrajectories>("/list_trajectories");
    if (!client->wait_for_service(1s)) return;
    auto req = std::make_shared<ListTrajectories::Request>();
    auto fut = client->async_send_request(req);
    // Asynchronous wait to not block ncurses
    std::thread([this, fut = std::move(fut)]() mutable {
      try {
        if (fut.wait_for(2s) == std::future_status::ready) {
          auto res = fut.get();
          std::lock_guard<std::mutex> lk(status_mu_);
          trajectories_.clear();
          for (size_t i = 0; i < res->names.size(); i++) {
            TrajectoryEntry e;
            e.name = res->names[i];
            e.description = i < res->descriptions.size() ? res->descriptions[i] : "";
            trajectories_.push_back(e);
          }
          traj_page_ = 0;
        }
      } catch (...) {
      }
    }).detach();
  }

  std::mutex status_mu_;  // 保护后台数据刷新

  // ────────────────── 输入分发 (核心防冲突逻辑) ──────────────────

  void handleInput(int ch) {
    if (input_mode_ != InputMode::NONE) {
      handleStringInput(ch);
      return;
    }

    if (takeover_mode_) {
      handleTakeoverInput(ch);
      return;
    }

    // 全局防冲突: 此时不在接管模式，也不在输入框，允许 M/T/C/G 切换
    handleGlobalInput(ch);
  }

  void handleGlobalInput(int ch) {
    // 退出
    if (ch == 'q' || ch == 'Q') {
      running_ = false;
      return;
    }

    // 视图切换
    if (ch == 'm' || ch == 'M') {
      view_ = View::STATE_MACHINE;
      return;
    }
    if (ch == 't' || ch == 'T') {
      view_ = View::TRAJECTORY;
      fetchTrajectories();
      return;
    }
    if (ch == 'h' || ch == 'H') {
      view_ = View::HELP;
      return;
    }

    // 需要接管的视图
    if (ch == 'c' || ch == 'C') {
      view_ = View::CARTESIAN;
      return;
    }
    if (ch == 'g' || ch == 'G') {
      view_ = View::GRIPPER;
      return;
    }

    // 视图内操作
    if (view_ == View::STATE_MACHINE)
      handleSMCommand(ch);
    else if (view_ == View::TRAJECTORY)
      handleTrajCommand(ch);
    else if (view_ == View::CARTESIAN || view_ == View::GRIPPER) {
      // 在这俩视图下，按 Enter 触发接管模式
      if (ch == '\n' || ch == '\r') {
        takeover_mode_ = true;
        logWarn("Entered Takeover mode. Press 'Esc' to release.");
      }
    }
  }

  void handleTakeoverInput(int ch) {
    // Esc 或者 q(仅在特定情况，最好严格约束Esc) 退出接管
    if (ch == 27) {  // 27 is Esc
      takeover_mode_ = false;
      log("Exited Takeover mode.", COLOR_PAIR_DEFAULT);
      return;
    }

    if (view_ == View::GRIPPER)
      handleGripperTakeover(ch);
    else if (view_ == View::CARTESIAN)
      handleCartesianTakeover(ch);
  }

  void handleStringInput(int ch) {
    // 字符串弹窗统一处理
    if (ch == 27) {  // Esc cancel
      input_mode_ = InputMode::NONE;
      log("Input canceled.");
      return;
    }

    if (input_mode_ == InputMode::DELETE_CONFIRM || input_mode_ == InputMode::OVERWRITE_CONFIRM) {
      // 单字符确认
      if (input_mode_ == InputMode::DELETE_CONFIRM)
        handleDeleteConfirm(ch);
      else if (input_mode_ == InputMode::OVERWRITE_CONFIRM)
        handleOverwrite(ch);
      return;
    }

    // 否则是字符累加
    if (ch == '\n' || ch == '\r') {
      std::string val = input_buffer_;
      input_buffer_.clear();
      if (input_mode_ == InputMode::SAVE_NAME)
        handleSaveName(val);
      else if (input_mode_ == InputMode::SAVE_DESC)
        handleSaveDesc(val);
    } else if (ch == KEY_BACKSPACE || ch == 127 || ch == '\b') {
      if (!input_buffer_.empty()) input_buffer_.pop_back();
    } else if (isprint(ch)) {
      // 限制名字不能有空格
      if (input_mode_ == InputMode::SAVE_NAME && ch == ' ') return;
      input_buffer_.push_back(ch);
    }
  }

  // ────────────────── 状态机操作 ──────────────────

  void handleSMCommand(int k) {
    if (k == 'e' || k == 'E')
      executeCurrentState();
    else if (k == 'x' || k == 'X')
      resetToIdle();
    else if (k == 'r' || k == 'R') {
      loadMissionSequence();
      logOk("Sequence reloaded from yaml.");
    }
  }

  void executeCurrentState() {
    if (executing_) {
      logWarn("Busy executing...");
      return;
    }
    if (current_idx_ >= states_.size()) {
      log("All states finished.");
      return;
    }

    auto& st = states_[current_idx_];
    if (st.trajectory.empty()) {
      if (current_idx_ + 1 < states_.size()) {
        current_idx_++;
        log("[" + st.id + "] Skipped (no traj) -> " + states_[current_idx_].id);
      }
      return;
    }

    executing_ = true;
    log("Exec [" + st.id + "] : " + st.trajectory + "...", COLOR_PAIR_HEADER);

    auto req = std::make_shared<LoadTrajectory::Request>();
    req->name = st.trajectory;
    req->execute = true;
    auto fut = load_client_->async_send_request(req);

    std::thread([this, fut = std::move(fut), id = st.id]() mutable {
      try {
        auto res = fut.get();
        if (res->success) {
          logOk("Success: " + id + " (" + std::to_string(res->duration) + "s)");
          std::lock_guard<std::mutex> lk(status_mu_);
          if (current_idx_ + 1 < states_.size()) current_idx_++;
        } else {
          logErr("Fail: " + id + " - " + res->message);
        }
      } catch (const std::exception& e) {
        logErr("Error: " + std::string(e.what()));
      }
      executing_ = false;
    }).detach();
  }

  void resetToIdle() {
    if (executing_) return;
    if (!reset_trajectory_.empty()) {
      executing_ = true;
      log("Resetting via: " + reset_trajectory_ + "...");
      auto req = std::make_shared<LoadTrajectory::Request>();
      req->name = reset_trajectory_;
      req->execute = true;
      auto fut = load_client_->async_send_request(req);
      std::thread([this, fut = std::move(fut)]() mutable {
        try {
          auto res = fut.get();
          if (res->success) {
            logOk("Reset OK.");
            std::lock_guard<std::mutex> lk(status_mu_);
            current_idx_ = 0;
          } else {
            logErr("Reset Fail: " + res->message);
          }
        } catch (...) {
        }
        executing_ = false;
      }).detach();
    } else {
      current_idx_ = 0;
      logOk("Reset to IDLE.");
    }
  }

  // ────────────────── 轨迹管理操作 ──────────────────

  void handleTrajCommand(int k) {
    if (k == 'r' || k == 'R') {
      fetchTrajectories();
      log("List refreshed.");
    } else if (k == 's' || k == 'S') {
      input_mode_ = InputMode::SAVE_NAME;
      input_buffer_.clear();
    } else if (k == 'n' || k == 'N') {
      size_t pages = (trajectories_.size() + PER_PAGE - 1) / PER_PAGE;
      if (pages > 1) traj_page_ = (traj_page_ + 1) % pages;
    } else if (k == 'p' || k == 'P') {
      size_t pages = (trajectories_.size() + PER_PAGE - 1) / PER_PAGE;
      if (pages > 1) traj_page_ = (traj_page_ + pages - 1) % pages;
    } else if (k == 'd' || k == 'D') {
      if (trajectories_.empty()) return;
      input_mode_ = InputMode::DELETE_CONFIRM;
    } else if (k >= '1' && k <= '9') {
      size_t idx = (k - '1') + traj_page_ * PER_PAGE;
      if (idx < trajectories_.size()) executeTraj(trajectories_[idx].name);
    }
  }

  void executeTraj(const std::string& name) {
    if (executing_) {
      logWarn("Busy...");
      return;
    }
    executing_ = true;
    log("Executing: " + name + "...");
    auto req = std::make_shared<LoadTrajectory::Request>();
    req->name = name;
    req->execute = true;
    auto fut = load_client_->async_send_request(req);
    std::thread([this, fut = std::move(fut), name]() mutable {
      try {
        auto res = fut.get();
        if (res->success)
          logOk("Done: " + name);
        else
          logErr("Fail: " + res->message);
      } catch (...) {
      }
      executing_ = false;
    }).detach();
  }

  void handleSaveName(const std::string& name) {
    if (name.empty()) {
      input_mode_ = InputMode::NONE;
      logWarn("Save canceled.");
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
    input_mode_ = exists ? InputMode::OVERWRITE_CONFIRM : InputMode::SAVE_DESC;
  }

  void handleSaveDesc(const std::string& desc) {
    doSave(pending_name_, desc);
    input_mode_ = InputMode::NONE;
  }

  void handleOverwrite(int k) {
    if (k == 'y' || k == 'Y') {
      input_mode_ = InputMode::SAVE_DESC;
    } else {
      input_mode_ = InputMode::NONE;
      logWarn("Overwrite canceled.");
    }
  }

  void doSave(const std::string& name, const std::string& desc) {
    log("Saving " + name + "...");
    auto req = std::make_shared<SaveLastTrajectory::Request>();
    req->name = name;
    req->description = desc;
    auto fut = save_client_->async_send_request(req);
    std::thread([this, fut = std::move(fut), name]() mutable {
      try {
        auto res = fut.get();
        if (res->success)
          logOk("Saved: " + name);
        else
          logErr("Save Fail: " + res->message);
      } catch (...) {
      }
      fetchTrajectories();
    }).detach();
  }

  void handleDeleteConfirm(int k) {
    if (k >= '1' && k <= '9') {
      size_t idx = (k - '1') + traj_page_ * PER_PAGE;
      if (idx < trajectories_.size()) {
        std::string path = trajectory_dir_ + "/" + trajectories_[idx].name + ".yaml";
        try {
          if (std::filesystem::exists(path)) {
            std::filesystem::remove(path);
            logOk("Deleted: " + trajectories_[idx].name);
            fetchTrajectories();
          }
        } catch (...) {
        }
      }
    }
    input_mode_ = InputMode::NONE;
  }

  // ────────────────── 接管操作：夹爪 ──────────────────

  void handleGripperTakeover(int ch) {
    if (ch == ' ') {
      gripper_torque_cmd_ = 0.0;
    }  // 瞬间归零
    else if (ch == '[') {
      gripper_torque_cmd_ -= 0.5;
    } else if (ch == ']') {
      gripper_torque_cmd_ += 0.5;
    } else if (ch == 's' || ch == 'S' || ch == '\n' || ch == '\r') {
      sendGripper(gripper_torque_cmd_);
    }
  }

  void sendGripper(double torque) {
    gripper_last_sent_ = torque;
    auto req = std::make_shared<GripperControl::Request>();
    req->torque = torque;
    auto fut = gripper_client_->async_send_request(req);

    // 在此处添加 mutable 关键字
    std::thread([this, fut = std::move(fut), torque]() mutable {
      try {
        auto res = fut.get();  // 现在可以正常调用 get()
        if (res->success) {
          logOk("Gripper cmd sent: " + std::to_string(torque));
        } else {
          logErr("Gripper fail: " + res->message);
        }
      } catch (const std::exception& e) {
        logErr("Gripper exception: " + std::string(e.what()));
      }
    }).detach();
  }

  // ────────────────── 接管操作：笛卡尔 (Jogging) ──────────────────

  void handleCartesianTakeover(int ch) {
    // 改变步长
    if (ch == '+' || ch == '=') {
      cartesian_step_ = std::min(1.0, cartesian_step_ + 0.01);
      return;
    }
    if (ch == '-') {
      cartesian_step_ = std::max(0.01, cartesian_step_ - 0.01);
      return;
    }

    double dx = 0, dy = 0, dz = 0, droll = 0, dpitch = 0, dyaw = 0;

    // 平移 W/S=X, A/D=Y, R/F=Z
    if (ch == 'w' || ch == 'W')
      dx = cartesian_step_;
    else if (ch == 's' || ch == 'S')
      dx = -cartesian_step_;
    else if (ch == 'a' || ch == 'A')
      dy = cartesian_step_;
    else if (ch == 'd' || ch == 'D')
      dy = -cartesian_step_;
    else if (ch == 'r' || ch == 'R')
      dz = cartesian_step_;
    else if (ch == 'f' || ch == 'F')
      dz = -cartesian_step_;

    // 姿态 方向键=Pitch/Yaw  Q/E=Roll
    else if (ch == KEY_UP)
      dpitch = cartesian_step_;
    else if (ch == KEY_DOWN)
      dpitch = -cartesian_step_;
    else if (ch == KEY_LEFT)
      dyaw = cartesian_step_;
    else if (ch == KEY_RIGHT)
      dyaw = -cartesian_step_;
    else if (ch == 'q')
      droll = -cartesian_step_;
    else if (ch == 'e')
      droll = cartesian_step_;
    else
      return;

    sendCartesianRelative(dx, dy, dz, droll, dpitch, dyaw);
  }

  void sendCartesianRelative(double dx, double dy, double dz, double droll, double dpitch,
                             double dyaw) {
    // 因为这是简单的示范实现，此处实际上应该是通知 cartesian_controller 提供一个增量 / relative
    // 服务或是话题. 若原设计 /move_to_cartesian_rpy 是绝对位置，我们需要在 /move_to_cartesian_rpy
    // 外再包装，或由 cartesian_controller 维持最新点。
    // 为了不大幅增加其他节点负担并发兼容，我们现要求服务能以现有位姿为基准规划 (或者我们在 UI
    // 层必须时刻订阅末端位姿)。 注：若后续需要，可在 cartesian_controller 新增 /jog_cartesian
    // 话题来支持无阻塞的丝滑手柄控制。
    logErr("Jogging feature relies on absolute position state or separate jog topic.");
    logWarn("For now, Cartesian jogging sends zero pos. Update cartesian node later.");
    // 占位功能，提醒用户更新。
  }

  // ────────────────── NCURSES UI 重绘 (全屏 10Hz) ──────────────────

  void drawUI() {
    erase();  // 清空缓存区
    int max_y, max_x;
    getmaxyx(stdscr, max_y, max_x);

    // 1. 顶部栏 F1-F4
    attron(COLOR_PAIR(COLOR_PAIR_HEADER));
    mvprintw(0, 0, "===================== ARV_V1 Mission Executor v4.0 =====================");
    mvprintw(1, 0, "|");
    attroff(COLOR_PAIR(COLOR_PAIR_HEADER));

    auto drawTab = [&](const std::string& name, View v) {
      if (view_ == v) attron(A_BOLD | COLOR_PAIR(COLOR_PAIR_HIGHLIGHT));
      printw(" %s ", name.c_str());
      if (view_ == v) attroff(A_BOLD | COLOR_PAIR(COLOR_PAIR_HIGHLIGHT));
      printw("  ");
    };
    mvprintw(1, 2, " ");
    drawTab("[M] StateMachine", View::STATE_MACHINE);
    drawTab("[T] Trajectory", View::TRAJECTORY);
    drawTab("[C] Cartesian", View::CARTESIAN);
    drawTab("[G] Gripper", View::GRIPPER);

    attron(COLOR_PAIR(COLOR_PAIR_HEADER));
    mvprintw(1, 71, "|");
    mvprintw(2, 0, "------------------------------------------------------------------------");
    attroff(COLOR_PAIR(COLOR_PAIR_HEADER));

    // 2. 接管模式全局提醒
    if (takeover_mode_) {
      attron(COLOR_PAIR(COLOR_PAIR_ERROR) | A_BOLD);
      mvprintw(3, 2, ">>> TAKEOVER MODE ACTIVE <<< [Esc] to exit. Global hotkeys M/T/C/G blocked.");
      attroff(COLOR_PAIR(COLOR_PAIR_ERROR) | A_BOLD);
    } else {
      if (view_ == View::CARTESIAN || view_ == View::GRIPPER) {
        attron(COLOR_PAIR(COLOR_PAIR_WARNING));
        mvprintw(3, 2, "Press [Enter] to takeover and control.");
        attroff(COLOR_PAIR(COLOR_PAIR_WARNING));
      } else {
        mvprintw(3, 2, "Global hotkeys active.");
      }
    }

    // 3. 核心视图区
    int cur_line = 5;
    std::lock_guard<std::mutex> lk(status_mu_);  // 防止访问后台数据时越界

    if (view_ == View::STATE_MACHINE) {
      mvprintw(cur_line++, 2, "Mission: %s - %s", mission_name_.c_str(), mission_desc_.c_str());
      cur_line++;
      for (size_t i = 0; i < states_.size(); i++) {
        if (i == current_idx_)
          attron(A_BOLD | COLOR_PAIR(COLOR_PAIR_WARNING));
        else if (i < current_idx_)
          attron(COLOR_PAIR(COLOR_PAIR_SUCCESS));
        else
          attron(A_DIM);

        mvprintw(cur_line++, 4, "[%s] %-16s %-16s %s %s",
                 (i < current_idx_ ? "*" : (i == current_idx_ ? ">" : " ")), states_[i].id.c_str(),
                 states_[i].trajectory.empty() ? "(No Traj)" : states_[i].trajectory.c_str(),
                 states_[i].description.c_str(), (i == current_idx_ ? " <- NOW" : ""));

        attroff(A_BOLD | COLOR_PAIR(COLOR_PAIR_WARNING));
        attroff(COLOR_PAIR(COLOR_PAIR_SUCCESS));
        attroff(A_DIM);
      }
      cur_line++;
      mvprintw(cur_line++, 2, "Hotkeys: [E] Execute Current   [X] Reset to IDLE   [R] Reload YAML");
    } else if (view_ == View::TRAJECTORY) {
      size_t pages = (trajectories_.size() + PER_PAGE - 1) / PER_PAGE;
      mvprintw(cur_line++, 2, "Database: %zu found (Page %zu/%zu)", trajectories_.size(),
               traj_page_ + 1, std::max((size_t)1, pages));
      cur_line++;
      size_t s = traj_page_ * PER_PAGE;
      size_t e = std::min(s + PER_PAGE, trajectories_.size());
      for (size_t i = s; i < e; i++) {
        mvprintw(cur_line++, 4, "[%zu] %-20s %s", (i - s + 1), trajectories_[i].name.c_str(),
                 trajectories_[i].description.c_str());
      }
      cur_line = 14;
      mvprintw(cur_line++, 2,
               "Hotkeys: [1-9] Execute   [S] Save Last   [D] Delete   [N/P] Next/Prev Page");
    } else if (view_ == View::CARTESIAN) {
      mvprintw(cur_line++, 2, "Cartesian Jogging Panel");
      cur_line++;
      attron(A_BOLD);
      mvprintw(cur_line++, 4, "Current Step Size: %.3f", cartesian_step_);
      attroff(A_BOLD);
      cur_line++;
      mvprintw(cur_line++, 4, "Translation: [W/S] X axis  [A/D] Y axis  [R/F] Z axis");
      mvprintw(cur_line++, 4, "Orientation: [Up/Down] Pitch  [Left/Right] Yaw  [Q/E] Roll");
      mvprintw(cur_line++, 4, "Config     : [+/-] Change Step Size");
    } else if (view_ == View::GRIPPER) {
      mvprintw(cur_line++, 2, "Gripper Direct Control");
      cur_line++;
      mvprintw(cur_line++, 4, "Last sent : ");
      attron(COLOR_PAIR(COLOR_PAIR_SUCCESS));
      printw("%.2f Nm", gripper_last_sent_);
      attroff(COLOR_PAIR(COLOR_PAIR_SUCCESS));

      mvprintw(cur_line++, 4, "Target (set) : ");
      attron(COLOR_PAIR(COLOR_PAIR_WARNING) | A_BOLD);
      printw("%.2f Nm", gripper_torque_cmd_);
      attroff(COLOR_PAIR(COLOR_PAIR_WARNING) | A_BOLD);

      cur_line++;
      mvprintw(cur_line++, 4, "Hotkeys (Takeover):");
      mvprintw(cur_line++, 6, "[ [ ] / [ ] ]  Adjust Target (-/+ 0.5)");
      mvprintw(cur_line++, 6, "[ SPACE ]      Zero Target (0.0)");
      mvprintw(cur_line++, 6, "[ s ]          Send Target Now");
    }

    // 4. 输入区
    cur_line = max_y - 7;
    attron(COLOR_PAIR(COLOR_PAIR_HEADER));
    mvprintw(cur_line++, 0,
             "------------------------------------------------------------------------");
    attroff(COLOR_PAIR(COLOR_PAIR_HEADER));

    if (input_mode_ == InputMode::SAVE_NAME) {
      mvprintw(cur_line, 2, "Enter Trajectory Name (no spaces): %s", input_buffer_.c_str());
    } else if (input_mode_ == InputMode::SAVE_DESC) {
      mvprintw(cur_line, 2, "Enter Description: %s", input_buffer_.c_str());
    } else if (input_mode_ == InputMode::DELETE_CONFIRM) {
      attron(COLOR_PAIR(COLOR_PAIR_ERROR));
      mvprintw(cur_line, 2, "Delete? Press [1-9] matching index, or [Esc] to cancel.");
      attroff(COLOR_PAIR(COLOR_PAIR_ERROR));
    } else if (input_mode_ == InputMode::OVERWRITE_CONFIRM) {
      attron(COLOR_PAIR(COLOR_PAIR_ERROR));
      mvprintw(cur_line, 2, "Name '%s' exists! Overwrite? [Y/N]", pending_name_.c_str());
      attroff(COLOR_PAIR(COLOR_PAIR_ERROR));
    } else {
      if (!takeover_mode_) mvprintw(cur_line, 2, "Press [H] for Help | [Q] to Quit");
    }

    // 5. 日志区
    cur_line++;
    auto logs = log_buffer_.get();
    for (const auto& l : logs) {
      attron(COLOR_PAIR(l.second));
      mvprintw(cur_line++, 0, "  %s", l.first.c_str());
      attroff(COLOR_PAIR(l.second));
    }

    refresh();  // 应用屏幕缓存
  }
};

int main(int argc, char** argv) {
  setlocale(LC_ALL, ""); // 使 ncurses 支持当前终端的字符集 (UTF-8)
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
