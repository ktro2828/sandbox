/**
 * @file double_subscriber.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "double_subscriber.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

ExactTimeSubscriber::ExactTimeSubscriber() : Node("exact_time_publisher")
{
  names_ = declear_parameter<std::vector<std::string>("names", names_);
  for (const auto & name : names_) {
    std::cout << name << std::endl;
  }
    temp1_sub_.subscribe(this, "temp_1");
    temp2_sub_.subscribe(this, "temp_2");

    message_filters::TimeSynchronizer<sensor_msgs::msg::Temperature, sensor_msgs::msg::Temperature> sync_(temp1_sub_, temp2_sub_, 3);
    sync_.registerCallback(std::bind(&topic_callback, this, _1, _2));
}

void ExactTimeSubscriber::topic_callback(const sensor_msgs::msg::Temperature::ConstSharedPtr tmp_1, const sensor_msgs::msg::Temperature::ConstSharedPtr tmp_2) const
{
    const char *temp_1 = std::to_string(tmp_1->temperature).c_str();
    const char *temp_2 = std::to_string(tmp_2->temperature).c_str();
    RCLCPP_INFO(this->get_logger(), "I heared: '%s' and '%s'" ,temp_1, temp_2);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExactTimeSubscriber>());
    rclcpp::shutdown();

    return 0;
}
