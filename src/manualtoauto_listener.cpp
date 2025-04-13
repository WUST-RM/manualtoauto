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
    : Node("ManualToAutoListener", options)
    {   
        this->declare_parameter<std::string>("csv_file_name", "odometry_data.csv");
        this->declare_parameter<std::string>("odometry_topic", "odometry");

        csv_file_name_ = this->get_parameter("csv_file_name").as_string();
        odometry_topic_ = this->get_parameter("odometry_topic").as_string();
   
        csv_file_.open(csv_file_name_, std::ios::out);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file!");
        } else {
     
            csv_file_ << "timestamp,linear_x,linear_y\n";
            csv_file_.flush();
        }

        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic_, 10,
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
       
        double linear_x = msg->twist.twist.linear.x;
        double linear_y = msg->twist.twist.linear.y;
    

       
        RCLCPP_INFO(this->get_logger(), "Current velocity: linear_x=%.2f m/s, linear_y=%.2f m/s",
                   linear_x, linear_y);


        if (csv_file_.is_open()) {
       
            auto timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            
            csv_file_ << std::to_string(timestamp) << ","
                     << linear_x << ","
                     << linear_y << "\n";
            csv_file_.flush();
        }
    }

    std::ofstream csv_file_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::string csv_file_name_;
    std::string odometry_topic_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ManualToAutoListener)
