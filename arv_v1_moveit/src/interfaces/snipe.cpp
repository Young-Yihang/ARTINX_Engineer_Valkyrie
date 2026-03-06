/**
 * @file snipe.cpp
 * @brief Vision-aiming serial node implementation (Boost.Asio async I/O)
 */
#include "snipe/snipe.hpp"
#include "snipe/Crc.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

SnipeNode::SnipeNode()
    : Node("snipe"),
      io_service_(),
      serial_(io_service_)
{
    // 声明串口参数
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baudrate", 115200);

    port_ = this->get_parameter("port").as_string();
    baud_rate_ = static_cast<int>(this->get_parameter("baudrate").as_int());

    // 打开串口
    try
    {
        serial_.open(port_);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        RCLCPP_INFO(this->get_logger(), "串口 %s 已打开，波特率 %d", port_.c_str(), baud_rate_);
    }
    catch (const boost::system::system_error &e)
    {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s", e.what());
        throw;
    }

    // 创建一个订阅者，订阅 "/localization_3d"（你可以修改为实际的话题名）
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/localization_3d", 10, std::bind(&SnipeNode::pose_callback, this, std::placeholders::_1));

    do_async_read();

    // 启动异步读取线程（保证 run() 时已有挂起的异步操作）
    io_thread_ = std::thread([this]()
                             { io_service_.run(); });
}

SnipeNode::~SnipeNode()
{
    io_service_.stop();
    if (io_thread_.joinable())
        io_thread_.join();
    if (serial_.is_open())
        serial_.close();
    RCLCPP_INFO(this->get_logger(), "SnipeNode 已关闭");
}

void SnipeNode::do_async_read()
{
    serial_.async_read_some(boost::asio::buffer(read_buf_, READ_BUF_SIZE),
                            std::bind(&SnipeNode::on_data_received, this, std::placeholders::_1, std::placeholders::_2));
}

void SnipeNode::on_data_received(const boost::system::error_code &ec, std::size_t bytes_transferred)
{
    if (!ec)
    {
        // // 处理接收到的数据
        // std::stringstream hex_stream;
        // for (size_t i = 0; i < bytes_transferred; ++i)
        // {
        //     hex_stream << std::hex << std::setw(2) << std::setfill('0') << (static_cast<int>(read_buf_[i]) & 0xFF) << " ";
        // }
        // RCLCPP_INFO(this->get_logger(), "接收到数据: %s", hex_stream.str().c_str());

        if (read_buf_[0] != SEASKY_SOF)
        {
            RCLCPP_INFO(this->get_logger(), "帧头错误: 0x%02X", read_buf_[0]);
            do_async_read();
            return;
        }

        uint16_t data_len = read_buf_[1] | (static_cast<uint16_t>(read_buf_[2]) << 8);
        size_t frame_len = data_len + SEASKY_OFFSET_BYTES;
        if (bytes_transferred < frame_len)
        {
            RCLCPP_INFO(this->get_logger(), "数据长度不足: %zu < %zu", bytes_transferred, frame_len);
            do_async_read();
            return;
        }

        if (!Crc::VerifyCrc8CheckSum(read_buf_, 4u))
        {
            RCLCPP_INFO(this->get_logger(), "CRC8 校验失败");
            do_async_read();
            return;
        }

        if (!Crc::VerifyCrc16CheckSum(read_buf_, static_cast<uint32_t>(frame_len)))
        {
            RCLCPP_INFO(this->get_logger(), "CRC16 校验失败");
            do_async_read();
            return;
        }

        // 解析帧
        uint16_t cmd_id, flags;

        cmd_id = read_buf_[4] | (static_cast<uint16_t>(read_buf_[5]) << 8);
        flags = read_buf_[6] | (static_cast<uint16_t>(read_buf_[7]) << 8);

        uint16_t payload_len = (data_len - 2) / 4;
        if (payload_len > MAX_FLOAT_LEN)
        {
            RCLCPP_INFO(this->get_logger(), "有效载荷长度超出限制: %u > %lu", payload_len, MAX_FLOAT_LEN);
            do_async_read();
            return;
        }

        std::memcpy(RecvData, &read_buf_[8], payload_len * sizeof(float));

        RCLCPP_INFO(this->get_logger(), "接收到帧: cmd_id=0x%04X, flags=0x%04X, yaw=%.3f,pitch=%.3f,roll=%.3f", cmd_id, flags, RecvData[0], RecvData[1], RecvData[2]);

        // 继续异步读取
        do_async_read();
    }
    else
    {
        if (ec != boost::asio::error::eof)
        {
            RCLCPP_INFO(this->get_logger(), "异步读取被中断: %s", ec.message().c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "串口连接已断开");
        }
    }
}

void SnipeNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // 提取位置 (xyz)
    x_ = msg->pose.position.x;
    y_ = msg->pose.position.y;
    z_ = msg->pose.position.z;

    // 将四元数转换为 tf2 四元数
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.orientation, quat);

    // 转换为欧拉角 (roll, pitch, yaw)
    tf2::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);

    // 打印结果
    RCLCPP_INFO(this->get_logger(), "Position: x=%.3f, y=%.3f, z=%.3f", x_, y_, z_);
    RCLCPP_INFO(this->get_logger(), "RPY (deg): roll=%.3f, pitch=%.3f, yaw=%.3f",
                roll_ * 180.0 / M_PI, pitch_ * 180.0 / M_PI, yaw_ * 180.0 / M_PI);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SnipeNode>();
    rclcpp::spin(node);
    return 0;
}