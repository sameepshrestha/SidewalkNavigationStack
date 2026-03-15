#ifndef FRODOBOT_MPPI_CRITICS__DIRECTION_CRITIC_HPP_
#define FRODOBOT_MPPI_CRITICS__DIRECTION_CRITIC_HPP_

#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

namespace frodobot_mppi_critics
{

class DirectionCritic : public mppi::critics::CriticFunction
{
public:
  void initialize() override;
  void score(mppi::CriticData & data) override;

protected:
  void headingCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  float weight_{0};
  float target_ux_{1.0};
  float target_uy_{0.0};
  bool received_heading_{false};

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr heading_sub_;
  std::mutex heading_mutex_;
};

} 

#endif
