#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <memory>
#include <sstream>

using namespace std::chrono_literals;

struct CsvData {
    double timestamp;
    geometry_msgs::msg::Twist twist;
};

class ManualToAutoPlayer : public rclcpp::Node
{
public:
    ManualToAutoPlayer(const rclcpp::NodeOptions & options)
    : Node("ManualToAutoPlayer", options)
    {
        
        this->declare_parameter<std::string>("csv_file", "/home/hy/wust_nav/odometry_data.csv");
        this->declare_parameter<bool>("loop", false);
        this->declare_parameter<double>("compensation_ratio", 1.2);
      
        std::string csv_file = this->get_parameter("csv_file").as_string();
        loop_ = this->get_parameter("loop").as_bool();
        compensation_ratio = this->get_parameter("compensation_ratio").as_double();

      
        clock_ = this->get_clock();
        rcl_clock_type_t clock_type = clock_->get_clock_type();
        use_system_time_ = (clock_type == RCL_SYSTEM_TIME);

        if (use_system_time_) {
            RCLCPP_INFO(this->get_logger(), "Using SYSTEM time.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Using ROS (sim) time.");
        }

  
        start_time_ = clock_->now();

   
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);


        if (!load_csv_data(csv_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load CSV file: %s", csv_file.c_str());
            return;
        }


        start_publishing_next();
    }

private:
    bool load_csv_data(const std::string & filename)
    {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return false;
        }

        std::string header;
        std::getline(file, header);

        std::string line;
        double first_timestamp = -1.0;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string timestamp_str, linear_x_str, linear_y_str;

            if (std::getline(iss, timestamp_str, ',') &&
                std::getline(iss, linear_x_str, ',') &&
                std::getline(iss, linear_y_str)) {

                try {
                    CsvData data;
                    data.timestamp = std::stod(timestamp_str);
                    if (first_timestamp < 0) {
                        first_timestamp = data.timestamp;
                    }
                    data.timestamp -= first_timestamp;

                    data.twist.linear.x = std::stod(linear_x_str);
                    data.twist.linear.y = std::stod(linear_y_str);
                    csv_data_.push_back(data);
                } catch (const std::exception & e) {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse line: %s", line.c_str());
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), 
                   "Loaded %zu velocity commands from CSV (time span: %.3fs)",
                   csv_data_.size(),
                   csv_data_.empty() ? 0.0 : csv_data_.back().timestamp);
        return !csv_data_.empty();
    }

    void start_publishing_next()
    {
        if (current_index_ >= csv_data_.size()) {
            if (loop_) {
                current_index_ = 0;
                start_time_ = clock_->now();
                RCLCPP_INFO(this->get_logger(), "Reached end of CSV, looping...");
            } else {
                RCLCPP_INFO(this->get_logger(), "Finished publishing all CSV data");
                auto stopvel=geometry_msgs::msg::Twist();
                stopvel.linear.x=0;
                stopvel.linear.y=0;
                publisher_->publish(stopvel);
                RCLCPP_INFO(this->get_logger(), "Publisher stop");
                rclcpp::shutdown();
                return;
            }
        }

        auto &data = csv_data_[current_index_];
        data.twist.linear.x *= compensation_ratio;
        data.twist.linear.y *= compensation_ratio;
        publisher_->publish(data.twist);

        RCLCPP_INFO(this->get_logger(), 
                   "Publishing cmd_vel[%zu/%zu]: linear.x=%.2f, linear.y=%.2f (t=%.3f)",
                   current_index_ + 1, csv_data_.size(),
                   data.twist.linear.x,
                   data.twist.linear.y,
                   data.timestamp);

        current_index_++;

        if (current_index_ >= csv_data_.size()) {
            // 设一个短延时决定是否继续
            timer_ = this->create_wall_timer(100ms, [this]() { start_publishing_next(); });
            return;
        }

        // 计算下一条与当前条之间的时间差
        double delay_sec = csv_data_[current_index_].timestamp - data.timestamp;
        auto delay = rclcpp::Duration::from_seconds(delay_sec);

        // 设置定时器触发下一条
        timer_ = this->create_wall_timer(delay.to_chrono<std::chrono::milliseconds>(), [this]() {
            timer_->cancel();  // 防止触发多次
            start_publishing_next();
        });
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<CsvData> csv_data_;
    size_t current_index_ = 0;
    bool loop_ = false;
    bool use_system_time_ = false;
    double compensation_ratio;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Time start_time_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ManualToAutoPlayer)
