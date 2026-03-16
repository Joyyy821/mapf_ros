/*********************************************************************
 *
 * MIT License
 *
 * Copyright (c) 2023 Junyi zhou
 * Copyright (c) 2025 Junyi zhou
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 *all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *********************************************************************/
#include <algorithm>
#include <iostream>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"

#include "mapf_msgs/msg/goal.hpp"

class GoalTransformer : public rclcpp::Node {
private:
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_goal_for_each_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_mapf_goal_init_;
  rclcpp::Publisher<mapf_msgs::msg::Goal>::SharedPtr pub_mapf_goal_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> goal_sub_arr_;

  // mapf params
  int agent_num_;
  std::string goal_for_each_topic_;
  std::vector<std::string> goal_topic_;
  std::vector<bool> goal_received_;

  mapf_msgs::msg::Goal goal_arr_;

  std::mutex goal_mtx;

public:
  GoalTransformer();
  ~GoalTransformer();

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal, size_t agent_idx);
  void goalForEachCallback(const geometry_msgs::msg::PoseArray::SharedPtr goals);

  void goalInitCallback(const std_msgs::msg::Bool::SharedPtr init);
  bool allGoalsReceived();
}; // class GoalTransformer

GoalTransformer::GoalTransformer() : Node("goal_transformer_node") {
  this->declare_parameter<int>("agent_num", 1);
  this->get_parameter("agent_num", agent_num_);

  goal_sub_arr_.resize(agent_num_);
  goal_topic_.resize(agent_num_);
  goal_received_.assign(agent_num_, false);
  goal_arr_.goal.poses.resize(agent_num_);

  // Legacy support: subscribe per-agent goal topics.
  for (int i = 0; i < agent_num_; ++i) {
    std::string goal_topic_param = "goal_topic.agent_" + std::to_string(i);
    this->declare_parameter<std::string>(goal_topic_param, "goal");
    this->get_parameter(goal_topic_param, goal_topic_[i]);

    goal_sub_arr_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_[i], 5, [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          goalCallback(msg, i);
        });
  }

  this->declare_parameter<std::string>("goal_for_each_topic", "goal_for_each");
  this->get_parameter("goal_for_each_topic", goal_for_each_topic_);
  sub_goal_for_each_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      goal_for_each_topic_, 5,
      std::bind(&GoalTransformer::goalForEachCallback, this, std::placeholders::_1));

  // subscribe goal init flag
  sub_mapf_goal_init_ = this->create_subscription<std_msgs::msg::Bool>(
      "goal_init_flag", 1,
      std::bind(&GoalTransformer::goalInitCallback, this, std::placeholders::_1));
  // pub goal in mapf form
  pub_mapf_goal_ = this->create_publisher<mapf_msgs::msg::Goal>("mapf_goal", 1);
}

GoalTransformer::~GoalTransformer() {}

void GoalTransformer::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal,
                                   size_t agent_idx) {
  std::lock_guard<std::mutex> lock(goal_mtx);
  goal_arr_.goal.header = goal->header;
  goal_arr_.goal.poses[agent_idx] = *goal;
  goal_received_[agent_idx] = true;
}

void GoalTransformer::goalForEachCallback(const geometry_msgs::msg::PoseArray::SharedPtr goals) {
  if (goals->poses.size() < static_cast<size_t>(agent_num_)) {
    RCLCPP_WARN(this->get_logger(),
                "Received %zu goals on '%s', but agent_num is %d.",
                goals->poses.size(), goal_for_each_topic_.c_str(), agent_num_);
    return;
  }

  std::lock_guard<std::mutex> lock(goal_mtx);
  goal_arr_.goal.header = goals->header;
  for (int i = 0; i < agent_num_; ++i) {
    goal_arr_.goal.poses[i].header = goals->header;
    goal_arr_.goal.poses[i].pose = goals->poses[i];
    goal_received_[i] = true;
  }
}

void GoalTransformer::goalInitCallback(const std_msgs::msg::Bool::SharedPtr init) {
  if (init->data) {
    std::lock_guard<std::mutex> lock(goal_mtx);
    if (!allGoalsReceived()) {
      const size_t ready_num = std::count(goal_received_.begin(), goal_received_.end(), true);
      RCLCPP_WARN(this->get_logger(),
                  "Ignored goal_init_flag=true because only %zu/%d goals are available.",
                  ready_num, agent_num_);
      return;
    }

    goal_arr_.header.stamp = this->get_clock()->now();
    goal_arr_.initial = true;
    pub_mapf_goal_->publish(goal_arr_);
  }
}

bool GoalTransformer::allGoalsReceived() {
  return std::all_of(goal_received_.begin(), goal_received_.end(), [](bool received) {
    return received;
  });
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalTransformer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
