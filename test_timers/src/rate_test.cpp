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


#include <memory>
#include <string>

#include <rclcpp/clock.hpp>
#include "rclcpp/duration.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace test_timers
{

class RateTest : public rclcpp::Node
{
  public:
    RateTest()
    : Node("rate_test"), count_(0), fast_measured_period_(0, 0), slow_measured_period_(0, 0)
    {
      slow_publisher_ = this->create_publisher<std_msgs::msg::String>("slow_topic", 10);
      fast_publisher_ = this->create_publisher<std_msgs::msg::String>("fast_topic", 10);
      previous_slow_time_ = this->wall_clock_.now();
      previous_fast_time_ = previous_slow_time_;
    }

    void slowUpdate()
    {
      auto message = std_msgs::msg::String();
      slow_measured_period_ =  this->wall_clock_.now() - previous_slow_time_;
      previous_slow_time_ = this->wall_clock_.now();
      message.data = "SLOOOOW message " + std::to_string(count_++) + " call period " + std::to_string(slow_measured_period_.seconds()) + " s, expected 1.0 s";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      slow_publisher_->publish(message);
    }

    void fastUpdate()
    {
      auto message = std_msgs::msg::String();
      fast_measured_period_ =  this->wall_clock_.now() - previous_fast_time_;
      previous_fast_time_ = this->wall_clock_.now();
      message.data = "FAST message " + std::to_string(count_++) + " call period " + std::to_string(fast_measured_period_.seconds()) + " s, expected 0.050 s";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      fast_publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr slow_publisher_, fast_publisher_;
    rclcpp::Time previous_fast_time_, previous_slow_time_;
    size_t count_;
    // wallclock to print time
    rclcpp::Clock wall_clock_;
    rclcpp::Duration fast_measured_period_, slow_measured_period_;
};
}  // namespace test_rates



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  auto node = std::make_shared<test_timers::RateTest>(); 

  executor->add_node(node);

  static int slowUpdateCount = 0;
  static int fastUpdateCount = 0;

  RCLCPP_INFO(node->get_logger(), "Spinning node: 'Rate'.");

  rclcpp::Rate rate(100);  // 100 hz => 10 ms 
  while(rclcpp::ok())
  {
      if (slowUpdateCount == 99) // 1000 ms => 1 Hz
      {
          slowUpdateCount = 0;
          node->slowUpdate();
      }
      else
      {
          slowUpdateCount++;
      }
      
      if (fastUpdateCount == 4) // 50 ms => 20 Hz
      {
        fastUpdateCount = 0;
        node->fastUpdate();
      }
      else
      {
        fastUpdateCount++;
      }
      executor->spin_some();
      rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
