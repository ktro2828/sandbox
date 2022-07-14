#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <chrono>
#include <memory>
#include <random>

using namespace std::chrono_literals;


class GridMapPublisher : public rclcpp::Node
{
public:
  GridMapPublisher(float map_length, float resolution) : Node("grid_map_publisher"), map_length_(map_length), resolution_(resolution)
  {
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("~/output/heatmap", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&GridMapPublisher::timer_callback, this));
    width_ = static_cast<int>(map_length / resolution_);
    height_ = static_cast<int>(map_length / resolution_);
  }

private:
  int width_;
  int height_;
  float map_length_;
  float resolution_;
  std::random_device rnd_;
  
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback()
  {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = "/map";
    msg.info.width = width_;
    msg.info.height = height_;
    msg.info.resolution = resolution_;
    msg.info.origin.position.x = -map_length_ / 2;
    msg.info.origin.position.y = -map_length_ / 2;
    msg.info.origin.orientation.w = 1;
    msg.info.origin.orientation.x = 0;
    msg.info.origin.orientation.y = 0;
    msg.info.origin.orientation.z = 0;
    msg.data.resize(width_ * height_);
    RCLCPP_INFO(this->get_logger(), "Start");
    for (int i=0; i<width_; ++i) {
      for (int j=0; j<height_; ++j) {
	RCLCPP_INFO(this->get_logger(), "%d", msg.data[i * j + j]);
	msg.data[i * j + j] += rnd_();
      }
    }
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published");
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMapPublisher>(100, 0.1));
  rclcpp::shutdown();
  return 0;
}
