// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//#include "test_timers/timer_wall_clock.hpp"

#include <memory>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"

using namespace std::chrono_literals;

namespace test_timers
{

class MyClock : public rclcpp::Node
{
  public:
    MyClock(int period_ms=100)
    : Node("my_clock")
    {
      this->declare_parameter("update_rate", 0.0);
      publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
      
      auto period = period_ms;
      double frequency = this->get_parameter("update_rate").as_double();
      if (frequency > 0.0)
        period = static_cast<int>(1000.0/frequency);

      timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period), std::bind(&MyClock::publish, this));
    }

    void publish()
    {
      auto now = this->get_clock()->now();
      auto message = rosgraph_msgs::msg::Clock();
      message.clock = now;
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
};
}  // namespace test_timers



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  const std::string node_name = "myclock";
  auto node = std::make_shared<test_timers::MyClock>(100); 

  executor->add_node(node);

  RCLCPP_INFO(node->get_logger(), "Spinning node: '%s'.", node_name.c_str());

  executor->spin();
  rclcpp::shutdown();
  return 0;
}
