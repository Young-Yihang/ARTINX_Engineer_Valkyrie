/**
 * @file mission_executor_node.cpp
 * @brief Application Layer Interaction Node
 *
 * Features:
 * - Persistent service connection (low latency)
 * - Dynamic mission loading via /list_trajectories
 * - ANSI TUI interface
 */

#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

#include "arv_v1_interfaces/srv/list_trajectories.hpp"
#include "arv_v1_interfaces/srv/load_trajectory.hpp"

using LoadTrajectory = arv_v1_interfaces::srv::LoadTrajectory;
using ListTrajectories = arv_v1_interfaces::srv::ListTrajectories;
using namespace std::chrono_literals;

struct Mission {
  std::string name;
  std::string description;
  char hotkey;
};

class MissionExecutorNode : public rclcpp::Node {
public:
  MissionExecutorNode() : Node("mission_executor") {
    // 1. Establish persistent connection for execution
    load_client_ = this->create_client<LoadTrajectory>("/load_trajectory");

    RCLCPP_INFO(this->get_logger(), "Connecting to services...");
    if (!load_client_->wait_for_service(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Service timeout");
      throw std::runtime_error("Connection failed");
    }

    // 2. Dynamically load available missions
    fetchMissions();

    RCLCPP_INFO(this->get_logger(), "Ready. Loaded %zu missions.",
                missions_.size());
  }

  void run() {
    while (rclcpp::ok()) {
      drawUI();

      // Blocking input for TUI interactiveness
      char key;
      std::cin >> key;

      handleInput(key);

      // Process background ROS callbacks
      rclcpp::spin_some(this->get_node_base_interface());
    }
  }

private:
  rclcpp::Client<LoadTrajectory>::SharedPtr load_client_;
  std::vector<Mission> missions_;
  std::string current_status_ = "Ready";
  std::mutex status_mutex_;

  void fetchMissions() {
    auto list_client =
        this->create_client<ListTrajectories>("/list_trajectories");

    if (!list_client->wait_for_service(2s)) {
      RCLCPP_WARN(this->get_logger(),
                  "List service unavailable, using defaults");
      setupDefaultMissions();
      return;
    }

    auto request = std::make_shared<ListTrajectories::Request>();
    auto future = list_client->async_send_request(request);

    // Blocking wait for list (startup only)
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      missions_.clear();

      // Check array consistency
      if (response->names.size() != response->descriptions.size()) {
        RCLCPP_WARN(this->get_logger(),
                    "Inconsistent trajectory data: %zu names vs %zu descriptions",
                    response->names.size(), response->descriptions.size());
      }

      // Map first 9 trajectories to keys '1'-'9'
      size_t count = std::min(response->names.size(), size_t(9));
      for (size_t i = 0; i < count; ++i) {
        missions_.push_back(
            {response->names[i],
             i < response->descriptions.size() && !response->descriptions[i].empty()
                 ? response->descriptions[i]
                 : "Trajectory " + std::to_string(i + 1),
             static_cast<char>('1' + i)});
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to fetch missions");
      setupDefaultMissions();
    }
  }

  void setupDefaultMissions() {
    missions_ = {{"home", "Return Home", '1'},
                 {"grab_cube", "Grab Cube", '2'},
                 {"bbb", "Test Trajectory B", '3'}};
  }

  void drawUI() {
    // ANSI Magic: \033[2J (Clear Screen) + \033[H (Home Cursor 0,0)
    // This creates a "repaint" effect without scrolling history
    std::cout << "\033[2J\033[H";

    std::cout << "=== ARV_V1 Mission Executor ===\n";
    std::cout << "Dynamic Loader | Persistent Client\n\n";
    std::cout << "Available Missions:\n";

    for (const auto &m : missions_) {
      std::cout << "  [" << m.hotkey << "] " << std::setw(20) << std::left
                << m.name << " : " << m.description << "\n";
    }

    std::cout << "\nControls: [R]efresh  [Q]uit\n";
    std::cout << "Status: ";
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      std::cout << current_status_;
    }
    std::cout << "\n> ";
    std::cout.flush();
  }

  void handleInput(char key) {
    if (key == 'q' || key == 'Q') {
      rclcpp::shutdown();
      return;
    }
    if (key == 'r' || key == 'R') {
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ = "Refreshing...";
      }
      fetchMissions();
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ = "List Refreshed";
      }
      return;
    }

    for (const auto &m : missions_) {
      if (m.hotkey == key) {
        executeMission(m.name);
        return;
      }
    }

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = "Unknown command";
    }
  }

  void executeMission(const std::string &name) {
    auto request = std::make_shared<LoadTrajectory::Request>();
    request->name = name;
    request->execute = true;

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = "Executing " + name + "...";
    }

    // Async call (non-blocking)
    auto future = load_client_->async_send_request(request);

    // Background Result Handler (use move capture for future)
    std::thread([this, future = std::move(future), name]() mutable {
      try {
        auto result = future.get();
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (result->success) {
          current_status_ = "Success: " + name;
        } else {
          current_status_ = "Failed: " + result->message;
        }
      } catch (const std::exception &e) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ = "Error: " + std::string(e.what());
      }
    }).detach();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MissionExecutorNode>();
    node->run();
  } catch (const std::exception &e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
