#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>
#include <memory>
#include <string>

using std::placeholders::_1;

class ManualToAutoListener : public rclcpp::Node
{
public:
    ManualToAutoListener(const rclcpp::NodeOptions & options)
    : Node("odometry_subscriber", options)
    {
        // 初始化CSV文件
        csv_file_.open("odometry_data.csv", std::ios::out);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file!");
        } else {
            // 写入CSV标题行
            csv_file_ << "timestamp,linear_x,linear_y\n";
            csv_file_.flush();
        }

        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10,
            std::bind(&ManualToAutoListener::odometry_callback, this, _1));
    }

    ~ManualToAutoListener() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 提取线速度
        double linear_x = msg->twist.twist.linear.x;
        double linear_y = msg->twist.twist.linear.y;
    

        // 输出到终端
        RCLCPP_INFO(this->get_logger(), "Current velocity: linear_x=%.2f m/s, linear_y=%.2f m/s",
                   linear_x, linear_y);

        // 存储到CSV文件
        if (csv_file_.is_open()) {
            // 使用消息时间戳（或当前时间）
            auto timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            
            csv_file_ << std::to_string(timestamp) << ","
                     << linear_x << ","
                     << linear_y << "\n";
            csv_file_.flush();
        }
    }

    std::ofstream csv_file_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ManualToAutoListener)