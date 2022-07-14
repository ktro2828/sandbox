/**
 * @file double_subscriber.hpp
 * @author Kotaro Uetake (kotaro.uetake@tier4.jp)
 * @brief 
 * @version 0.1
 * @date 2022-04-18
 * 
 * @copyright Copyright (c) 2022
 * @ref https://answers.ros.org/question/361637/using-c-message-filters-in-ros2/
 * 
 */
#ifndef DOUBLE_SUBSCRIBER_HPP_
#define DOUBLE_SUBSCRIBER_HPP_

#include <memory>
#include <string>
#include <cstring>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

class ExactTimeSubscriber : public rclcpp::Node
{
    public:
    /**
     * @brief Construct a new Exact Time Subscriber object
     * 
     */
    ExactTimeSubscriber();

    private:
    /**
     * @brief 
     * 
     * @param tmp_1 
     * @param tmp_2 
     */
    void topic_callback(const sensor_msgs::msg::Temperature::ConstSharedPtr tmp_1, const sensor_msgs::msg::Temperature::ConstSharedPtr tmp_2) const;

    message_filters::Subscriber<sensor_msgs::msg::Temperature> temp1_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Temperature> temp2_sub_;
  std::vector<std::string> names_{"hoge", "fuga", "hogehoge"};
};     // class ExactTimeSubscriber

#endif // DOUBLE_SUBSCRIBER_HPP_
