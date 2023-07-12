#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <fstream>


bool has_realtime_kernel() {
  std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
  bool has_realtime = false;
  if (realtime_file.is_open()) {
    realtime_file >> has_realtime;
  }
  return has_realtime;
}


bool configure_sched_fifo(int priority) {
  struct sched_param schedp;
  memset(&schedp, 0, sizeof(schedp));
  schedp.sched_priority = priority;
  return !sched_setscheduler(0, SCHED_FIFO, &schedp);
}


class InterpolationNode : public rclcpp::Node {
 public:
  InterpolationNode() : Node("interpolation_node") {
    subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/servo_node/commands", 10,
        std::bind(&InterpolationNode::callback, this, std::placeholders::_1));

    publisher_ =
        create_publisher<std_msgs::msg::Float64MultiArray>("/yu_position_controller/commands", 10);

    // Initialize the interpolation buffer with zero values and timestamps
    for (size_t i = 0; i < buffer_size_; ++i) {
      buffer_[i].resize(num_axes_ + 1);
      for (size_t j = 0; j < num_axes_; ++j) {
        buffer_[i][j] = 0.0;
      }
      buffer_[i][num_axes_] = 0.0;  // Initialize timestamp to zero
    }
    RCLCPP_INFO(this->get_logger(), "Interpolation buffer initialized");
    // Timer to republish the interpolated positions at 500 Hz
    timer_ = create_wall_timer(std::chrono::microseconds(2000),
                               std::bind(&InterpolationNode::timerCallback, this));
  }

 private:
  void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // Store the incoming message in the buffer along with its timestamp
    // RCLCPP_INFO(this->get_logger(), "Received command: '%f'", msg->data[0]);
    buffer_[buffer_index_] = msg->data;
    buffer_[buffer_index_].push_back(rclcpp::Clock().now().seconds());

    // Increment the buffer index
    buffer_index_ = (buffer_index_ + 1) % buffer_size_;
    if (callback_count_ < buffer_size_) {
      callback_count_++;
    }
  }

  void timerCallback() {
    // only publish commands if the buffer is valid.
    if (callback_count_ < buffer_size_) {
      return;
    }
    auto current_time = rclcpp::Clock().now().seconds();

    auto most_recent_cmd_time =
        buffer_[(buffer_index_ - 1 + buffer_size_) % buffer_size_][num_axes_];

    if (current_time - most_recent_cmd_time > 0.1) {
      // wait for the buffer to be filled again with new cmds.
      callback_count_ = 0;
    }

    auto interpolation_time = current_time - 0.012;  // Interpolate some time in the past

    std_msgs::msg::Float64MultiArray interpolated_msg;
    interpolated_msg.data.resize(num_axes_);

    // Find the previous and next buffer indices for interpolation by looping through the pose
    // buffer
    size_t prev_index = buffer_index_;
    size_t next_index = (buffer_index_ + 1) % buffer_size_;
    double next_time = buffer_[next_index][num_axes_];

    while (interpolation_time > next_time) {
      // in case we looped through the whole buffer and the most recent pose is still to old, stick
      // with it
      if ((next_index + 1) % buffer_size_ == buffer_index_) {
        return;
      }
      prev_index = next_index;
      next_index = (next_index + 1) % buffer_size_;
      next_time = buffer_[next_index][num_axes_];
    }

    double prev_time = buffer_[prev_index][num_axes_];

    double diff = next_time - prev_time;
    double elapsed = interpolation_time - prev_time;
    double interpolation_ratio = elapsed / diff;

    // Clamp the interpolation ratio between 0 and 1
    interpolation_ratio = std::max(0.0, std::min(1.0, interpolation_ratio));

    // Interpolate the position value
    for (size_t axis = 0; axis < num_axes_; ++axis) {
      double prev_value = buffer_[prev_index][axis];
      double next_value = buffer_[next_index][axis];
      interpolated_msg.data[axis] = prev_value + (next_value - prev_value) * interpolation_ratio;
    }
    // Publish the interpolated positions
    publisher_->publish(interpolated_msg);
  }

  static constexpr size_t num_axes_ = 6;
  static constexpr size_t buffer_size_ = 10;
  std::vector<double> buffer_[buffer_size_];
  size_t buffer_index_ = 0;

  u_int callback_count_ = 0;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node = std::make_shared<InterpolationNode>();
  executor->add_node(node);

  if (has_realtime_kernel()) {
    if (!configure_sched_fifo(40)) {
      RCLCPP_WARN(node->get_logger(), "Could not enable FIFO RT scheduling policy");
    } 
    
  }
  else{
      RCLCPP_WARN(node->get_logger(), "Could not enable FIFO RT scheduling policy");
    }
  executor->spin();
  rclcpp::shutdown();
  return 0;
}