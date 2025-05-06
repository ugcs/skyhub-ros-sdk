#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace skyhub_demo {
namespace time {


class SkyhubTimer
{
  public:
  using SharedPtr = std::shared_ptr<SkyhubTimer>;
  using UniquePtr = std::unique_ptr<SkyhubTimer>;

  SkyhubTimer(rclcpp_lifecycle::LifecycleNode& node,
              std::chrono::milliseconds period,
              std::function<void()> cbk)
  : m_node(node), m_cbk(cbk), m_period(period)
  {
    m_cb_timer_group = m_node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  void start() {
    m_timer = m_node.create_wall_timer(
      m_period, m_cbk, m_cb_timer_group);
  }
  void stop() {
    m_timer = nullptr;
  }

  protected:
  rclcpp_lifecycle::LifecycleNode& m_node;
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::CallbackGroup::SharedPtr m_cb_timer_group;
  std::function<void()> m_cbk;
  std::chrono::milliseconds m_period;
};

}}
