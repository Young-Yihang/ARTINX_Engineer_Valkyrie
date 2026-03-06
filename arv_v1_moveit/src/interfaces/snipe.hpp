/**
 * @file snipe.hpp
 * @brief Vision-aiming serial node — Boost.Asio async RX + Seasky protocol
 */
#ifndef SNIPE_SNIPE_HPP
#define SNIPE_SNIPE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <cstdint>
#include <string>
#include <memory>
#include <thread>

static constexpr uint8_t SEASKY_SOF = 0xA5;
static constexpr size_t SEASKY_OFFSET_BYTES = 8;
static constexpr size_t MAX_FLOAT_LEN = 16;

class SnipeNode : public rclcpp::Node
{
public:
    SnipeNode();
    ~SnipeNode();

private:
    // 串口相关
    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_;
    std::thread io_thread_;
    std::string port_;
    int baud_rate_;

    // 异步读取数据回调
    void do_async_read();
    void on_data_received(const boost::system::error_code& ec, std::size_t bytes_transferred);

    // 缓冲区
    static const int READ_BUF_SIZE = 256;
    uint8_t read_buf_[READ_BUF_SIZE];

    float RecvData[MAX_FLOAT_LEN];

    double x_, y_, z_;
    double roll_, pitch_, yaw_;

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};


#endif