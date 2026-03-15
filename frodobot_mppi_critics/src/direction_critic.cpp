#include "frodobot_mppi_critics/direction_critic.hpp"

#include <xtensor/xmath.hpp>
#include <xtensor/xarray.hpp>

namespace frodobot_mppi_critics
{

void DirectionCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(weight_, "weight", 15.0f);
  
  std::string heading_topic;
  getParam(heading_topic, "heading_topic", std::string("/goal_heading"));
  
  RCLCPP_INFO(logger_, "DirectionCritic initialized. Topic: %s, Weight: %.2f", heading_topic.c_str(), weight_);

  auto node = parent_.lock();
  heading_sub_ = node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    heading_topic, 1, 
    std::bind(&DirectionCritic::headingCallback, this, std::placeholders::_1));
}

void DirectionCritic::headingCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(heading_mutex_);
  target_ux_ = msg->vector.x;
  target_uy_ = msg->vector.y;
  received_heading_ = true;
}

void DirectionCritic::score(mppi::CriticData & data)
{
  if (!enabled_ || !received_heading_) {
    return;
  }

  std::lock_guard<std::mutex> lock(heading_mutex_);
  float ux = target_ux_;
  float uy = target_uy_;

  auto & x = data.trajectories.x;
  auto & y = data.trajectories.y;

  auto start_x = xt::view(x, xt::all(), 0);
  auto start_y = xt::view(y, xt::all(), 0);

  // dx, dy are (batch, time)
  auto dx = x - xt::view(start_x, xt::all(), xt::newaxis());
  auto dy = y - xt::view(start_y, xt::all(), xt::newaxis());

  auto dist = xt::sqrt(dx * dx + dy * dy);
  
  // To avoid division by zero
  auto alignment = (dx * ux + dy * uy) / (dist + 0.001f);

  // Mean alignment over time steps (axis 1)
  auto avg_alignment = xt::mean(alignment, {1});

  // Calculate cost. Max alignment is 1.0 (0 cost). Min alignment is -1.0 (2*weight cost).
  // Cost = weight * (1.0 - avg_alignment)
  auto cost = weight_ * (1.0f - avg_alignment);

  data.costs += cost;
}

}  // namespace frodobot_mppi_critics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(frodobot_mppi_critics::DirectionCritic, mppi::critics::CriticFunction)
