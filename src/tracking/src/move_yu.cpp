#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tf2_ros/buffer.h"

tf2::Transform calcDifferenceTransform(const tf2::Transform &a, const tf2::Transform &b)
{
  tf2::Transform difference = tf2::Transform();
  difference.setOrigin(b.getOrigin() - a.getOrigin());
  difference.setRotation(b.getRotation() * a.getRotation().inverse());
  return difference;
}

class AverageFilter
{
public:
  AverageFilter()
      : windowSize(3),
        ringIndex(0),
        filteredValue(tf2::Vector3(0.0, 0.0, 0.0)),
        window(windowSize, tf2::Vector3(0.0, 0.0, 0.0)) {}
  AverageFilter(int windowSize)
      : windowSize(windowSize),
        ringIndex(0),
        filteredValue(tf2::Vector3(0.0, 0.0, 0.0)),
        window(windowSize, tf2::Vector3(0.0, 0.0, 0.0)) {}

  tf2::Vector3 update(const tf2::Vector3 &newValue)
  {
    window[ringIndex] = newValue;
    ringIndex = (ringIndex + 1) % windowSize;

    auto sum = tf2::Vector3(0.0, 0.0, 0.0);

    for (int i = 0; i < windowSize; i++)
    {
      sum += window[i];
    }

    filteredValue = sum / windowSize;

    return filteredValue;
  }

  tf2::Vector3 getFilteredValue() { return filteredValue; }

  void reset() { std::fill(window.begin(), window.end(), tf2::Vector3(0.0, 0.0, 0.0)); }

private:
  int windowSize = 1;
  int ringIndex = 0;
  tf2::Vector3 filteredValue = tf2::Vector3();
  std::vector<tf2::Vector3> window = std::vector<tf2::Vector3>(windowSize, tf2::Vector3());
};

class MoveRobotNode : public rclcpp::Node
{
public:
  MoveRobotNode()
      : Node("move_robot_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_),
        average_filter_linear_(6),
        average_filter_angular_(6)
  {
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);
    ee_velocity_publisher_ =
        this->create_publisher<geometry_msgs::msg::TwistStamped>("/ee_velocity", 10);
    ee_transform_publisher_ =
        this->create_publisher<geometry_msgs::msg::TransformStamped>("/ee_transform", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                     std::bind(&MoveRobotNode::timerCallback, this));

    start_time_ = this->get_clock()->now();
  }

private:
  void timerCallback()
  {
    try
    {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped =
          tf_buffer_.lookupTransform("base_link", "wrist3", tf2::TimePointZero);

      ee_transform_publisher_->publish(transform_stamped);

      double y = transform_stamped.transform.translation.y;
      tf2::Vector3 deviation;
      tf2::fromMsg(transform_stamped.transform.translation, deviation);
      deviation -= tf2::Vector3(0.61, 0.0, 0.66);
      deviation *= -0.1;

      deviation.setY(calculateVelocity(y));

      geometry_msgs::msg::TwistStamped velocity;
      velocity.header.frame_id = "base_link";
      velocity.header.stamp = this->get_clock()->now();
      velocity.twist.linear = tf2::toMsg(deviation);
      velocity_publisher_->publish(velocity);

      last_velocity_ = velocity.twist.linear.y;

      tf2::Transform this_transform;
      tf2::fromMsg(transform_stamped.transform, this_transform);

      auto difference = calcDifferenceTransform(last_transform_, this_transform);

      tf2::Vector3 ee_velocity = difference.getOrigin() * 100.0;
      tf2::Vector3 ee_angular_velocity =
          difference.getRotation().getAxis() * difference.getRotation().getAngle() * 100.0;

      ee_velocity = average_filter_linear_.update(ee_velocity);
      ee_angular_velocity = average_filter_angular_.update(ee_angular_velocity);

      geometry_msgs::msg::TwistStamped ee_velocity_msg;
      ee_velocity_msg.header.frame_id = "base_link";
      ee_velocity_msg.header.stamp = this->get_clock()->now();
      ee_velocity_msg.twist.linear = tf2::toMsg(ee_velocity);
      ee_velocity_msg.twist.angular = tf2::toMsg(ee_angular_velocity);
      ee_velocity_publisher_->publish(ee_velocity_msg);

      last_transform_ = this_transform;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
    }
  }

  double calculateVelocity(double y)
  {
    double dt = (this->get_clock()->now() - start_time_).seconds();
    double max_velocity = 0.05 * dt; // Maximum velocity
    double acceleration = 0.05 * dt; // Acceleration magnitude

    if (y < -0.10)
      direction_ = 1.0;

    else if (y > 0.10)
      direction_ = -1.0;

    if (direction_ > 0.0)
      return std::min(last_velocity_ + acceleration * 0.01 * direction_, max_velocity * direction_);

    else
      return std::max(last_velocity_ + acceleration * 0.01 * direction_, max_velocity * direction_);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ee_velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr ee_transform_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double last_velocity_ = 0.0;
  double direction_ = 1.0;

  tf2::Transform last_transform_;
  AverageFilter average_filter_linear_;
  AverageFilter average_filter_angular_;

  rclcpp::Time start_time_ = rclcpp::Time(0, 0);
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveRobotNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}