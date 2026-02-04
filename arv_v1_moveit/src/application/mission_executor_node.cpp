/**
 * @file mission_executor_node.cpp
 * @brief Application Layer Interaction Node v2.0
 *
 * Features:
 * - Persistent service connection (low latency)
 * - Dynamic mission loading via /list_trajectories
 * - Trajectory saving (interactive input)
 * - Trajectory deletion
 * - Trajectory info display
 * - ANSI TUI interface with colors
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
#include <thread>
#include <vector>

#include "arv_v1_interfaces/srv/list_trajectories.hpp"
#include "arv_v1_interfaces/srv/load_trajectory.hpp"
#include "arv_v1_interfaces/srv/save_last_trajectory.hpp"

using LoadTrajectory = arv_v1_interfaces::srv::LoadTrajectory;
using ListTrajectories = arv_v1_interfaces::srv::ListTrajectories;
using SaveLastTrajectory = arv_v1_interfaces::srv::SaveLastTrajectory;
using namespace std::chrono_literals;

// ANSI Color codes
const std::string COLOR_RED = "\033[0;31m";
const std::string COLOR_GREEN = "\033[0;32m";
const std::string COLOR_YELLOW = "\033[1;33m";
const std::string COLOR_BLUE = "\033[0;34m";
const std::string COLOR_CYAN = "\033[0;36m";
const std::string COLOR_RESET = "\033[0m";
const std::string COLOR_BOLD = "\033[1m";

struct Mission {
  std::string name;
  std::string description;
  char hotkey;
};

class MissionExecutorNode : public rclcpp::Node {
public:
  MissionExecutorNode() : Node("mission_executor") {
    // Get trajectory directory
    trajectory_dir_ =
        std::string(getenv("HOME")) + "/ros2_ws/src/arv_v1_moveit/config/trajectories";

    // 1. Establish persistent connections
    load_client_ = this->create_client<LoadTrajectory>("/load_trajectory");
    save_client_ = this->create_client<SaveLastTrajectory>("/save_last_trajectory");

    RCLCPP_INFO(this->get_logger(), "Connecting to services...");
    if (!load_client_->wait_for_service(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Service timeout");
      throw std::runtime_error("Connection failed");
    }

    if (!save_client_->wait_for_service(2s)) {
      RCLCPP_WARN(this->get_logger(), "Save service not available (optional)");
    }

    // 2. Dynamically load available missions
    fetchMissions();
  }

  void run() {
    while (rclcpp::ok()) {
      drawUI();

      // Read input based on mode
      std::string input;
      if (input_mode_ == InputMode::COMMAND) {
        // Single character mode
        char key;
        std::cin >> key;
        input = std::string(1, key);
      } else {
        // Line input mode
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::getline(std::cin, input);
      }

      handleInput(input);

      // Small delay to prevent CPU spinning
      rclcpp::spin_some(this->get_node_base_interface());
    }
  }

private:
  // Service clients
  rclcpp::Client<LoadTrajectory>::SharedPtr load_client_;
  rclcpp::Client<SaveLastTrajectory>::SharedPtr save_client_;

  // Data
  std::vector<Mission> missions_;
  std::string current_status_ = "Ready";
  std::mutex status_mutex_;
  std::string trajectory_dir_;

  // Pagination
  static constexpr size_t MISSIONS_PER_PAGE = 6;
  size_t current_page_ = 0;

  // Input state machine
  enum class InputMode {
    COMMAND,
    SAVE_NAME,
    SAVE_DESC,
    DELETE_CONFIRM,
    INFO_SELECT,
    OVERWRITE_CONFIRM
  };
  InputMode input_mode_ = InputMode::COMMAND;

  // Temporary input buffers
  std::string pending_name_;
  std::string pending_desc_;
  std::string target_mission_;

  void fetchMissions() {
    auto list_client = this->create_client<ListTrajectories>("/list_trajectories");

    if (!list_client->wait_for_service(2s)) {
      RCLCPP_WARN(this->get_logger(), "List service unavailable");
      missions_.clear();
      return;
    }

    auto request = std::make_shared<ListTrajectories::Request>();
    auto future = list_client->async_send_request(request);

    // Blocking wait for list (startup only)
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      missions_.clear();

      // Check array consistency
      if (response->names.size() != response->descriptions.size()) {
        RCLCPP_WARN(this->get_logger(),
                    "Inconsistent trajectory data: %zu names vs %zu descriptions",
                    response->names.size(), response->descriptions.size());
      }

      // Load all trajectories (pagination handled in UI)
      for (size_t i = 0; i < response->names.size(); ++i) {
        Mission m;
        m.name = response->names[i];
        m.description = response->descriptions[i];
        m.hotkey = '0';  // Will be assigned dynamically per page
        missions_.push_back(m);
      }

      // Reset to first page
      current_page_ = 0;
    }
  }

  void drawUI() {
    // Clear screen and home cursor
    std::cout << "\033[2J\033[H";

    // Title with color
    std::cout << COLOR_CYAN << COLOR_BOLD;
    std::cout << COLOR_RESET << "\n";
    std::cout << "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀  ⠀⠀⠀⠀⠀⢀⣀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀" << std::endl;
    std::cout << "⠀⠀⠀⠀⠀⠀⠀⠀⠀  ⠀⠀⠀⠀⠀⢀⣾⣿⣿⣿⣿⣦⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀" << std::endl;
    std::cout << "⠀⠀⠀⠀⠀⠀⠀⠀⠀  ⠀⠀⠀⠀⠀⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣶⣦⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀" << std::endl;
    std::cout << "⠀⠀⠀⠀⠀⠀⠀⠀  ⠀⠀⠀⠀⠀⣠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣶⣶⣦⣤⣄⡀⠀⠀" << std::endl;
    std::cout << "⠀⠀⠀⠀⠀⠀⠀⠀  ⠀⠀⠀⢀⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⠀" << std::endl;
    std::cout << "⠀⠀⠀⠀⠀⠀⠀  ⠀⠀⢀⣴⣿⣿⣿⣿⣿⣿⠿⠁⣸⣿⣿⣿⣿⣿⡿⢻⣿⣿⣿⣿⣿⣿⣿⣿⣿⡏⠀" << std::endl;
    std::cout << "⠀⠀⠀⠀⠀⠀   ⠀⠀⠲⢿⣿⣿⠿⠿⠟⢋⠹⢰⠂⣿⣿⣿⣿⣿⡿⢃⡘⢿⣿⠿⣿⡿⢿⣿⣿⣿⠃⠀" << std::endl;
    std::cout << "⠀⠀⠀⠀⠀   ⠀⠀⠀⢠⣾⣿⡟⢠⡄⣀⣀⡀⠉⣆⣃⠹⣿⣿⡿⢡⣶⣽⣦⠁⢀⠀⡀⠈⣿⣿⠏⠀⠀" << std::endl;
    std::cout << "⠀⠀⠀⠀⢀⠤⠄⠒⠠⠤⢀⡉⠻⡇⢸⢁⣿⣿⡟⣼⣿⣿⣷⣬⣩⣄⠉⠙⠛⠿⢀⠈⠛⠁⠀⣽⣿⠀⠀⠀" << std::endl;
    std::cout << "⠀⠀⢀⠎⠁⠀⢀⡀⠀⠀⠀⠉⢢⡀⢸⣧⣛⣫⠴⠿⠿⠿⢿⣿⣿⡟⣸⣶⣦⠄⣀⠀⣲⣶⣿⢿⣿⠀⠀⠀" << std::endl;
    std::cout << "⠀⢀⠇⢀⢀⣶⣿⣿⡇⠀⠀⠀⠀⠱⡀⠛⣁⠤⠖⠒⣒⣲⡤⣌⠙⢧⠻⣿⠟⣰⠏⣸⣿⣿⡏⢸⣿⠀⠀⠀" << std::endl;
    std::cout << "⠀⡸⠀⡀⣾⣿⣿⢫⢠⠄⠀⠀⠀⠀⠑⠊⠀⠀⣾⣿⣿⡿⣛⣻⣷⡄⠹⣶⡾⢋⣴⣿⣿⣿⠇⣼⡏⠀⠀⠀" << std::endl;
    std::cout << "⠀⡇⠀⠇⣿⣿⣿⢈⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⢻⣿⣿⣷⣻⣿⣯⣿⡄⢹⠄⠉⣹⣿⣿⡿⢠⣿⡇⠀⠀⠀" << std::endl;
    std::cout << "⠀⢸⠀⠐⢜⢿⣿⢎⠄⠀⠀⠀⠀⠀⠀⠀⠀⠈⠸⢿⣿⣿⣿⣿⣿⣿⡇⠀⢀⣼⣿⣿⣿⢁⣾⣿⠀⠀⠀⠀" << std::endl;
    std::cout << "⠀⠈⢆⠀⠀⣑⣶⡦⣀⢄⠀⠀⠀⠀⡠⠔⢚⣉⣉⣉⣉⣉⣉⣉⠛⠻⠁⡘⠛⣿⣿⡿⢁⣾⣿⡇⠀⠀⠀⠀" << std::endl;
    std::cout << "⠀⠀⠈⢮⢿⣿⣿⣝⠿⢇⣧⣠⡀⣸⠀⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣄⠁⢞⠿⢋⣴⣿⣿⣿⡇⠀⠀⠀⠀" << std::endl;
    std::cout << "⠀⠀⣰⣄⠑⢽⣿⣿⣿⣿⣿⣿⣿⢹⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⠀⠀⡿⣿⣿⠿⣿⣇⠀⠀⠀⠀" << std::endl;
    std::cout << "⠀⠰⢿⣿⣷⣤⡙⠻⢿⣿⣿⣿⣿⣿⡆⢹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠇⠀⠙⢏⡞⠀⠘⠺⠄⠀⠀⠀" << std::endl;
    std::cout << "⠀⠀⠀⠈⠉⠛⠻⠷⢤⠈⠉⠙⠛⠛⠓⠀⢿⣿⣿⣿⣿⣿⣿⠿⠟⢋⠽⠋⠀⠀⠀⠀⠁⠀⠀⠀⠀⠀⠀⠀" << std::endl;
    std::cout << "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠉⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀     " << std::endl;

    // Missions list with pagination
    if (missions_.empty()) {
      std::cout << COLOR_YELLOW << "  No missions found. Press [S] to save a trajectory.\n"
                << COLOR_RESET;
    } else {
      size_t total_pages = (missions_.size() + MISSIONS_PER_PAGE - 1) / MISSIONS_PER_PAGE;
      size_t start_idx = current_page_ * MISSIONS_PER_PAGE;
      size_t end_idx = std::min(start_idx + MISSIONS_PER_PAGE, missions_.size());

      std::cout << COLOR_BOLD << "Available Missions (Page " << (current_page_ + 1) << "/"
                << total_pages << "):" << COLOR_RESET << "\n";

      for (size_t i = start_idx; i < end_idx; ++i) {
        char hotkey = '1' + (i - start_idx);
        std::cout << "  [" << COLOR_YELLOW << hotkey << COLOR_RESET << "] ";
        std::cout << std::setw(20) << std::left << missions_[i].name;
        std::cout << " : " << missions_[i].description << "\n";
      }
    }

    std::cout << "\n";

    // Commands with pagination
    std::cout << COLOR_BOLD << "Commands:\n" << COLOR_RESET;
    if (missions_.size() > MISSIONS_PER_PAGE) {
      std::cout << "  [N]ext  [P]rev  |  [S]ave  [D]elete  [I]nfo  [R]efresh  [H]elp  [Q]uit\n\n";
    } else {
      std::cout << "  [S]ave  [D]elete  [I]nfo  [R]efresh  [H]elp  [Q]uit\n\n";
    }

    // Status with color
    std::cout << "Status: ";
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      if (current_status_.find("Error") != std::string::npos ||
          current_status_.find("Failed") != std::string::npos) {
        std::cout << COLOR_RED << current_status_ << COLOR_RESET;
      } else if (current_status_.find("Success") != std::string::npos ||
                 current_status_.find("Saved") != std::string::npos) {
        std::cout << COLOR_GREEN << current_status_ << COLOR_RESET;
      } else if (current_status_.find("Executing") != std::string::npos) {
        std::cout << COLOR_CYAN << current_status_ << COLOR_RESET;
      } else {
        std::cout << current_status_;
      }
    }
    std::cout << "\n";

    // Input prompt based on mode
    switch (input_mode_) {
      case InputMode::COMMAND:
        std::cout << "\n> ";
        break;
      case InputMode::SAVE_NAME:
        std::cout << "\n" << COLOR_YELLOW << "Enter trajectory name (no spaces): " << COLOR_RESET;
        break;
      case InputMode::SAVE_DESC:
        std::cout << "\n" << COLOR_YELLOW << "Enter description: " << COLOR_RESET;
        break;
      case InputMode::DELETE_CONFIRM:
        std::cout << "\n"
                  << COLOR_RED << "Select mission to delete [1-9] or [C]ancel: " << COLOR_RESET;
        break;
      case InputMode::INFO_SELECT:
        std::cout << "\n"
                  << COLOR_CYAN << "Select mission for info [1-9] or [C]ancel: " << COLOR_RESET;
        break;
      case InputMode::OVERWRITE_CONFIRM:
        std::cout << "\n"
                  << COLOR_YELLOW << "Mission '" << pending_name_
                  << "' exists. Overwrite? [Y/N]: " << COLOR_RESET;
        break;
    }

    std::cout.flush();
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
      case InputMode::INFO_SELECT:
        handleInfoSelect(input.empty() ? '\0' : input[0]);
        break;
      case InputMode::OVERWRITE_CONFIRM:
        handleOverwriteConfirm(input.empty() ? '\0' : input[0]);
        break;
    }
  }

  void handleCommand(char key) {
    if (key == 'q' || key == 'Q') {
      rclcpp::shutdown();
      return;
    }

    // Pagination - Next page
    if (key == 'n' || key == 'N') {
      if (missions_.size() > MISSIONS_PER_PAGE) {
        size_t total_pages = (missions_.size() + MISSIONS_PER_PAGE - 1) / MISSIONS_PER_PAGE;
        current_page_ = (current_page_ + 1) % total_pages;
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ =
            "Page " + std::to_string(current_page_ + 1) + "/" + std::to_string(total_pages);
      }
      return;
    }

    // Pagination - Previous page
    if (key == 'p' || key == 'P') {
      if (missions_.size() > MISSIONS_PER_PAGE) {
        size_t total_pages = (missions_.size() + MISSIONS_PER_PAGE - 1) / MISSIONS_PER_PAGE;
        current_page_ = (current_page_ + total_pages - 1) % total_pages;
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ =
            "Page " + std::to_string(current_page_ + 1) + "/" + std::to_string(total_pages);
      }
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
        current_status_ = "Mission list refreshed";
      }
      return;
    }

    if (key == 's' || key == 'S') {
      input_mode_ = InputMode::SAVE_NAME;
      return;
    }

    if (key == 'd' || key == 'D') {
      if (missions_.empty()) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ = "No missions to delete";
        return;
      }
      input_mode_ = InputMode::DELETE_CONFIRM;
      return;
    }

    if (key == 'i' || key == 'I') {
      if (missions_.empty()) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ = "No missions to show info";
        return;
      }
      input_mode_ = InputMode::INFO_SELECT;
      return;
    }

    if (key == 'h' || key == 'H') {
      showHelp();
      return;
    }

    // Check if it's a mission hotkey (1-6 for current page)
    if (key >= '1' && key <= '9') {
      size_t idx = (key - '1') + (current_page_ * MISSIONS_PER_PAGE);
      if (idx < missions_.size()) {
        executeMission(missions_[idx].name);
        return;
      }
    }

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = "Unknown command. Press [H] for help";
    }
  }

  void executeMission(const std::string& name) {
    auto request = std::make_shared<LoadTrajectory::Request>();
    request->name = name;
    request->execute = true;

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = "Executing " + name + "...";
    }

    auto future = load_client_->async_send_request(request);

    std::thread([this, future = std::move(future), name]() mutable {
      try {
        auto result = future.get();
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (result->success) {
          current_status_ = "Success: " + name + " (" + std::to_string(result->duration) + "s)";
        } else {
          current_status_ = "Failed: " + result->message;
        }
      } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ = "Error: " + std::string(e.what());
      }
    }).detach();
  }

  void handleSaveName(const std::string& name) {
    // Validate name
    if (name.empty()) {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = "Error: Name cannot be empty";
      input_mode_ = InputMode::COMMAND;
      return;
    }

    // Check for invalid characters
    if (name.find('/') != std::string::npos || name.find('\\') != std::string::npos ||
        name.find(' ') != std::string::npos) {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = "Error: Invalid characters (no spaces or slashes)";
      input_mode_ = InputMode::COMMAND;
      return;
    }

    // Check if mission exists
    if (missionExists(name)) {
      pending_name_ = name;
      input_mode_ = InputMode::OVERWRITE_CONFIRM;
      return;
    }

    pending_name_ = name;
    input_mode_ = InputMode::SAVE_DESC;
  }

  void handleSaveDesc(const std::string& desc) {
    pending_desc_ = desc;
    saveTrajectory(pending_name_, pending_desc_);
    input_mode_ = InputMode::COMMAND;
  }

  void handleOverwriteConfirm(char key) {
    if (key == 'y' || key == 'Y') {
      input_mode_ = InputMode::SAVE_DESC;
    } else {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = "Save cancelled";
      input_mode_ = InputMode::COMMAND;
    }
  }

  void saveTrajectory(const std::string& name, const std::string& desc) {
    if (!save_client_->service_is_ready()) {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = "Error: Save service not available";
      return;
    }

    auto request = std::make_shared<SaveLastTrajectory::Request>();
    request->name = name;
    request->description = desc;

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = "Saving " + name + "...";
    }

    auto future = save_client_->async_send_request(request);

    std::thread([this, future = std::move(future), name]() mutable {
      try {
        auto result = future.get();
        std::lock_guard<std::mutex> lock(status_mutex_);

        if (result->success) {
          current_status_ = "Saved: " + name;
          // Auto-refresh mission list
          fetchMissions();
        } else {
          current_status_ = "Save failed: " + result->message;
        }
      } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ = "Save error: " + std::string(e.what());
      }
    }).detach();
  }

  void handleDeleteConfirm(char key) {
    if (key == 'c' || key == 'C') {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = "Delete cancelled";
      input_mode_ = InputMode::COMMAND;
      return;
    }

    if (key >= '1' && key <= '9') {
      // Map hotkey to actual mission index based on current page
      size_t idx = (key - '1') + (current_page_ * MISSIONS_PER_PAGE);
      if (idx < missions_.size()) {
        deleteTrajectory(missions_[idx].name);
      } else {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ = "Invalid mission number";
      }
    }

    input_mode_ = InputMode::COMMAND;
  }

  void deleteTrajectory(const std::string& name) {
    std::string path = trajectory_dir_ + "/" + name + ".yaml";

    try {
      if (std::filesystem::exists(path)) {
        std::filesystem::remove(path);
        {
          std::lock_guard<std::mutex> lock(status_mutex_);
          current_status_ = "Deleted: " + name;
        }
        fetchMissions();
      } else {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ = "Error: File not found";
      }
    } catch (const std::filesystem::filesystem_error& e) {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = "Error: " + std::string(e.what());
    }
  }

  void handleInfoSelect(char key) {
    if (key == 'c' || key == 'C') {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = "Info cancelled";
      input_mode_ = InputMode::COMMAND;
      return;
    }

    if (key >= '1' && key <= '9') {
      // Map hotkey to actual mission index based on current page
      size_t idx = (key - '1') + (current_page_ * MISSIONS_PER_PAGE);
      if (idx < missions_.size()) {
        showMissionInfo(missions_[idx].name);
      } else {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ = "Invalid mission number";
      }
    }

    input_mode_ = InputMode::COMMAND;
  }

  void showMissionInfo(const std::string& name) {
    std::string path = trajectory_dir_ + "/" + name + ".yaml";

    try {
      YAML::Node config = YAML::LoadFile(path);

      std::cout << "\n" << COLOR_CYAN;
      std::cout << "╔════════════════════════════════════════════════════════════╗\n";
      std::cout << "║  Trajectory Information                                    ║\n";
      std::cout << "╚════════════════════════════════════════════════════════════╝\n";
      std::cout << COLOR_RESET;

      std::cout << "\n  " << COLOR_BOLD << "Name:        " << COLOR_RESET << name << "\n";

      if (config["meta"]) {
        if (config["meta"]["description"]) {
          std::string desc = config["meta"]["description"].as<std::string>();
          std::cout << "  " << COLOR_BOLD << "Description: " << COLOR_RESET << desc << "\n";
        }

        if (config["meta"]["duration_sec"]) {
          double duration = config["meta"]["duration_sec"].as<double>();
          std::cout << "  " << COLOR_BOLD << "Duration:    " << COLOR_RESET << std::fixed
                    << std::setprecision(2) << duration << " seconds\n";
        }

        if (config["meta"]["saved_at"]) {
          std::string saved_at = config["meta"]["saved_at"].as<std::string>();
          std::cout << "  " << COLOR_BOLD << "Saved at:    " << COLOR_RESET << saved_at << "\n";
        }
      }

      if (config["points"]) {
        int point_count = config["points"].size();
        std::cout << "  " << COLOR_BOLD << "Points:      " << COLOR_RESET << point_count << "\n";
      }

      if (config["start_position"]) {
        std::cout << "  " << COLOR_BOLD << "Start pos:   " << COLOR_RESET;
        for (const auto& pos : config["start_position"]) {
          std::cout << std::fixed << std::setprecision(2) << pos.as<double>() << " ";
        }
        std::cout << "\n";
      }

      std::cout << "\n" << COLOR_YELLOW << "Press Enter to continue..." << COLOR_RESET;
      std::cin.get();

    } catch (const std::exception& e) {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = "Error reading file: " + std::string(e.what());
    }
  }

  void showHelp() {
    std::cout << "\n" << COLOR_CYAN << COLOR_BOLD;
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                      HELP MENU                             ║\n";
    std::cout << "╠════════════════════════════════════════════════════════════╣\n";
    std::cout << COLOR_RESET;
    std::cout << "║  " << COLOR_YELLOW << "[1-9]" << COLOR_RESET
              << "  Execute selected trajectory                         ║\n";
    std::cout << "║  " << COLOR_YELLOW << "[S]" << COLOR_RESET
              << "    Save last executed trajectory from RViz           ║\n";
    std::cout << "║         → Will prompt for name and description            ║\n";
    std::cout << "║  " << COLOR_YELLOW << "[D]" << COLOR_RESET
              << "    Delete a trajectory                               ║\n";
    std::cout << "║  " << COLOR_YELLOW << "[I]" << COLOR_RESET
              << "    Show trajectory information                       ║\n";
    std::cout << "║  " << COLOR_YELLOW << "[R]" << COLOR_RESET
              << "    Refresh mission list                              ║\n";
    std::cout << "║  " << COLOR_YELLOW << "[H]" << COLOR_RESET
              << "    Show this help menu                               ║\n";
    std::cout << "║  " << COLOR_YELLOW << "[Q]" << COLOR_RESET
              << "    Quit program                                       ║\n";
    std::cout << COLOR_CYAN << COLOR_BOLD;
    std::cout << "╠════════════════════════════════════════════════════════════╣\n";
    std::cout << "║  Workflow: Plan in RViz → Execute → Press [S] to save     ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n";
    std::cout << COLOR_RESET;
    std::cout << "\n" << COLOR_YELLOW << "Press Enter to continue..." << COLOR_RESET;
    std::cin.get();
  }

  bool missionExists(const std::string& name) const {
    for (const auto& m : missions_) {
      if (m.name == name) {
        return true;
      }
    }
    return false;
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
