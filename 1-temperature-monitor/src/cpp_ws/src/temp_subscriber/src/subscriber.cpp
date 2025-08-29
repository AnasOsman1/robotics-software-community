#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>
#include <functional>
#include "sensor_interface/msg/temp_data.hpp"
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>
class TempSubscriber : public rclcpp::Node 
{
public:
    explicit TempSubscriber(const std::string & node_name = "temp_subscriber")
    : Node(node_name), window_size_(10), sample_count_(0)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        subscriber_ = this->create_subscription<sensor_interface::msg::TempData>(
            "temp_publisher", qos,
            std::bind(&TempSubscriber::callBackFunction, this, std::placeholders::_1));
        
        temp_samples_.reserve(window_size_);
        RCLCPP_INFO(this->get_logger(), "Temperature Subscriber started. Collecting batches of %zu samples...", window_size_);
    }

private:
    void callBackFunction(const sensor_interface::msg::TempData::SharedPtr msg)
    {
        // Add temperature to persistent vector
        temp_samples_.push_back(msg->temperature);
        sample_count_++;
        
        // When we have 10 samples, process the batch
        if (temp_samples_.size() == window_size_) {
            processBatch(msg);
            temp_samples_.clear(); // Clear for next batch
        }
    }
    
    void processBatch(const sensor_interface::msg::TempData::SharedPtr msg)
    {
        // Calculate statistics
        auto min_it = std::min_element(temp_samples_.begin(), temp_samples_.end());
        auto max_it = std::max_element(temp_samples_.begin(), temp_samples_.end());
        double min_temp = *min_it;  // Dereference iterator
        double max_temp = *max_it;  // Dereference iterator
        double average = std::accumulate(temp_samples_.begin(), temp_samples_.end(), 0.0) / window_size_;
        double temp = msg->temperature;  // Current temperature
        double range = max_temp - min_temp;
        RCLCPP_INFO(this->get_logger(), 
                    "\n=== BATCH COMPLETE ===\n"
                    "Current temp: %.2f °%s\n"
                    "Batch Stats:\n"
                    "  Min:     %.2f °%s\n"
                    "  Max:     %.2f °%s\n"
                    "  Average: %.2f °%s\n"
                    "  Range:   %.2f °%s\n"
                    "Total samples: %zu\n"
                    "======================",
                    static_cast<double>(msg->temperature), msg->unit.c_str(),
                    min_temp, msg->unit.c_str(),
                    max_temp, msg->unit.c_str(),
                    average, msg->unit.c_str(),
                    max_temp - min_temp, msg->unit.c_str(),
                    sample_count_);
        // Check if current temperature is within normal ranges
        if (temp < min_temp || temp  > max_temp) {
            RCLCPP_WARN(this->get_logger(), 
                        "Temperature out of range: Min=%.2f °%s, Max=%.2f °%s",
                        min_temp, msg->unit.c_str(), max_temp, msg->unit.c_str());
        }

        if(range > 10.0){
            RCLCPP_WARN(this->get_logger(), "High temprature variation detected: %.2f °%s", range, msg->unit.c_str());
        }
        // Extreme temperature check
        if(min_temp < -40 || max_temp > 80){
            RCLCPP_FATAL(this->get_logger(),
                "Extreme temperature detected: Min=%.2f °%s, Max=%.2f °%s",
                min_temp, msg->unit.c_str(), max_temp, msg->unit.c_str());
        }

        // Check for NaN or invalid temperature values
        if(std::isnan(msg->temperature) || std::isinf(msg->temperature)){
            RCLCPP_ERROR(this->get_logger(),"Received invalid temperature data.");
        }

    }

    rclcpp::Subscription<sensor_interface::msg::TempData>::SharedPtr subscriber_;
    std::vector<double> temp_samples_;  // Member variable - persists across callbacks
    const size_t window_size_;          // Batch size
    size_t sample_count_;               // Total samples received
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TempSubscriber>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    node.reset();
    RCLCPP_INFO(rclcpp::get_logger("Shutting down"), "completed.");
    
    return 0;
}