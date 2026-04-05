#include "waypoint_manager/action_chainer.hpp"

#include <chrono>
#include <thread>
#include <cmath>

using namespace std::chrono_literals;

namespace waypoint_manager
{

// ---------------------------------------------------------------------------
// Helper: declare-or-get parameter (matches amr_motion_control_2wd safeParam)
// ---------------------------------------------------------------------------
template <typename T>
static T safeParam(rclcpp::Node::SharedPtr node,
                   const std::string & name, T default_val)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, default_val);
  }
  return node->get_parameter(name).get_value<T>();
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
ActionChainer::ActionChainer(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // Read action server names from parameters
  std::string spin_server = safeParam(node_, "action_chainer.spin_server",
    std::string("spin"));
  std::string translate_server = safeParam(node_, "action_chainer.translate_server",
    std::string("amr_motion_translate"));
  std::string translate_reverse_server = safeParam(node_,
    "action_chainer.translate_reverse_server",
    std::string("amr_translate_reverse_action"));
  std::string turn_server = safeParam(node_, "action_chainer.turn_server",
    std::string("turn"));
  std::string yaw_server = safeParam(node_, "action_chainer.yaw_control_server",
    std::string("amr_motion_yaw_control"));

  action_timeout_sec_ = safeParam(node_, "action_chainer.action_timeout_sec", 60.0);
  spin_speed_ = safeParam(node_, "segment_planner.default_spin_speed", 40.0);
  spin_accel_ = safeParam(node_, "segment_planner.default_spin_accel", 30.0);
  turn_accel_angle_ = safeParam(node_, "segment_planner.default_turn_accel_angle", 15.0);

  // Create action clients
  spin_client_ = rclcpp_action::create_client<SpinAction>(node_, spin_server);
  translate_client_ = rclcpp_action::create_client<TranslateAction>(node_, translate_server);
  translate_reverse_client_ = rclcpp_action::create_client<TranslateAction>(
    node_, translate_reverse_server);
  turn_client_ = rclcpp_action::create_client<TurnAction>(node_, turn_server);
  yaw_client_ = rclcpp_action::create_client<YawControlAction>(node_, yaw_server);

  RCLCPP_INFO(node_->get_logger(),
    "[ActionChainer] Clients: spin='%s', translate='%s', translate_rev='%s', "
    "turn='%s', yaw='%s'",
    spin_server.c_str(), translate_server.c_str(),
    translate_reverse_server.c_str(), turn_server.c_str(), yaw_server.c_str());
}

// ---------------------------------------------------------------------------
// execute() — start executing a sequence of segments (called from detached thread)
// ---------------------------------------------------------------------------
void ActionChainer::execute(
  const std::vector<waypoint_interfaces::msg::Segment> & segments,
  OnComplete on_complete, OnFeedback on_feedback)
{
  segments_ = segments;
  on_complete_ = on_complete;
  on_feedback_ = on_feedback;
  current_index_ = 0;
  total_distance_ = 0.0;
  completed_count_ = 0;
  start_time_ = node_->now();
  state_.store(ChainerState::EXECUTING);

  executeNext();
}

// ---------------------------------------------------------------------------
// cancel()
// ---------------------------------------------------------------------------
void ActionChainer::cancel()
{
  state_.store(ChainerState::CANCELLED);

  // Cancel any active action clients
  if (spin_client_) { spin_client_->async_cancel_all_goals(); }
  if (translate_client_) { translate_client_->async_cancel_all_goals(); }
  if (translate_reverse_client_) { translate_reverse_client_->async_cancel_all_goals(); }
  if (turn_client_) { turn_client_->async_cancel_all_goals(); }
  if (yaw_client_) { yaw_client_->async_cancel_all_goals(); }

  // Cancel wait timer if active
  if (wait_timer_) {
    wait_timer_->cancel();
    wait_timer_.reset();
  }
}

// ---------------------------------------------------------------------------
// pause() / resume()
// ---------------------------------------------------------------------------
void ActionChainer::pause()
{
  if (state_.load() == ChainerState::EXECUTING ||
      state_.load() == ChainerState::WAITING_RESULT) {
    state_.store(ChainerState::PAUSED);
  }
}

void ActionChainer::resume()
{
  if (state_.load() == ChainerState::PAUSED) {
    state_.store(ChainerState::EXECUTING);
  }
}

// ---------------------------------------------------------------------------
// executeNext() — advance to the next segment
// ---------------------------------------------------------------------------
void ActionChainer::executeNext()
{
  // Check cancellation
  if (state_.load() == ChainerState::CANCELLED) {
    ChainResult result;
    result.status = -1;  // cancelled
    result.completed_segments = completed_count_;
    result.total_distance = total_distance_;
    result.elapsed_time = (node_->now() - start_time_).seconds();
    if (on_complete_) { on_complete_(result); }
    return;
  }

  // Check pause — spin-wait
  while (state_.load() == ChainerState::PAUSED) {
    std::this_thread::sleep_for(50ms);
    if (state_.load() == ChainerState::CANCELLED) {
      ChainResult result;
      result.status = -1;
      result.completed_segments = completed_count_;
      result.total_distance = total_distance_;
      result.elapsed_time = (node_->now() - start_time_).seconds();
      if (on_complete_) { on_complete_(result); }
      return;
    }
  }

  // Check if all segments done
  if (current_index_ >= segments_.size()) {
    state_.store(ChainerState::MISSION_DONE);
    ChainResult result;
    result.status = 0;  // success
    result.completed_segments = completed_count_;
    result.total_distance = total_distance_;
    result.elapsed_time = (node_->now() - start_time_).seconds();
    if (on_complete_) { on_complete_(result); }
    return;
  }

  const auto & seg = segments_[current_index_];

  // Publish feedback
  if (on_feedback_) {
    ChainFeedback fb;
    fb.current_segment_id = seg.segment_id;
    fb.segment_action_type = seg.action_type;
    fb.segment_phase = 0;
    fb.current_speed = 0.0;
    fb.distance_remaining = 0.0;
    fb.waypoint_id = seg.waypoint_to;
    on_feedback_(fb);
  }

  state_.store(ChainerState::WAITING_RESULT);

  using Seg = waypoint_interfaces::msg::Segment;
  switch (seg.action_type) {
    case Seg::SPIN:
      executeSpin(seg);
      break;
    case Seg::TRANSLATE:
      executeTranslate(seg);
      break;
    case Seg::TRANSLATE_REVERSE:
      executeTranslateReverse(seg);
      break;
    case Seg::TURN:
      executeTurn(seg);
      break;
    case Seg::YAWCTRL:
      executeYawControl(seg);
      break;
    case Seg::WAIT:
      executeWait(seg);
      break;
    default:
      RCLCPP_WARN(node_->get_logger(),
        "[ActionChainer] Unknown action_type=%d, skipping segment %u",
        seg.action_type, seg.segment_id);
      onSegmentComplete(0);
      break;
  }
}

// ---------------------------------------------------------------------------
// executeSpin()
// ---------------------------------------------------------------------------
void ActionChainer::executeSpin(const waypoint_interfaces::msg::Segment & seg)
{
  if (!spin_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node_->get_logger(), "[ActionChainer] Spin server not available");
    onSegmentComplete(-3);
    return;
  }

  auto goal = SpinAction::Goal();
  goal.target_angle = seg.spin_angle;  // deg, absolute map frame
  goal.max_angular_speed = spin_speed_;
  goal.angular_acceleration = spin_accel_;
  goal.hold_steer = seg.hold_steer;
  goal.exit_steer_angle = seg.exit_steer_angle;

  segment_done_.store(false);
  segment_status_ = 0;
  segment_distance_ = 0.0;

  auto send_goal_options = rclcpp_action::Client<SpinAction>::SendGoalOptions();

  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<SpinAction>::WrappedResult & wrapped) {
      std::lock_guard<std::mutex> lock(result_mutex_);
      segment_status_ = wrapped.result->status;
      segment_done_.store(true);
    };

  send_goal_options.feedback_callback =
    [this, &seg](
      rclcpp_action::ClientGoalHandle<SpinAction>::SharedPtr,
      const std::shared_ptr<const SpinAction::Feedback> fb) {
      if (on_feedback_) {
        ChainFeedback cfb;
        cfb.current_segment_id = seg.segment_id;
        cfb.segment_action_type = seg.action_type;
        cfb.segment_phase = fb->phase;
        cfb.current_speed = fb->current_speed;
        cfb.distance_remaining = 0.0;
        cfb.waypoint_id = seg.waypoint_to;
        on_feedback_(cfb);
      }
    };

  auto goal_future = spin_client_->async_send_goal(goal, send_goal_options);

  // Polling loop — runs on detached thread, compatible with SingleThreadedExecutor
  auto deadline = node_->now() + rclcpp::Duration::from_seconds(action_timeout_sec_);
  while (!segment_done_.load() && rclcpp::ok()) {
    if (state_.load() == ChainerState::CANCELLED) {
      spin_client_->async_cancel_all_goals();
      return;
    }
    if (node_->now() > deadline) {
      RCLCPP_WARN(node_->get_logger(),
        "[ActionChainer] Spin timeout for segment %u", seg.segment_id);
      spin_client_->async_cancel_all_goals();
      onSegmentComplete(-3);
      return;
    }
    std::this_thread::sleep_for(10ms);
  }

  onSegmentComplete(segment_status_);
}

// ---------------------------------------------------------------------------
// executeTranslate()
// ---------------------------------------------------------------------------
void ActionChainer::executeTranslate(const waypoint_interfaces::msg::Segment & seg)
{
  if (!translate_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node_->get_logger(), "[ActionChainer] Translate server not available");
    onSegmentComplete(-3);
    return;
  }

  auto goal = TranslateAction::Goal();
  goal.start_x = seg.start_x;
  goal.start_y = seg.start_y;
  goal.end_x = seg.end_x;
  goal.end_y = seg.end_y;
  goal.max_linear_speed = seg.max_linear_speed;
  goal.acceleration = seg.acceleration;
  goal.exit_speed = seg.exit_speed;
  goal.has_next = seg.has_next;
  goal.control_mode = seg.control_mode;
  goal.hold_steer = seg.hold_steer;
  goal.exit_steer_angle = seg.exit_steer_angle;

  segment_done_.store(false);
  segment_status_ = 0;
  segment_distance_ = 0.0;

  auto send_goal_options = rclcpp_action::Client<TranslateAction>::SendGoalOptions();

  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<TranslateAction>::WrappedResult & wrapped) {
      std::lock_guard<std::mutex> lock(result_mutex_);
      segment_status_ = wrapped.result->status;
      segment_distance_ = wrapped.result->actual_distance;
      segment_done_.store(true);
    };

  send_goal_options.feedback_callback =
    [this, &seg](
      rclcpp_action::ClientGoalHandle<TranslateAction>::SharedPtr,
      const std::shared_ptr<const TranslateAction::Feedback> fb) {
      if (on_feedback_) {
        double total_dist = std::hypot(seg.end_x - seg.start_x, seg.end_y - seg.start_y);
        ChainFeedback cfb;
        cfb.current_segment_id = seg.segment_id;
        cfb.segment_action_type = seg.action_type;
        cfb.segment_phase = fb->phase;
        cfb.current_speed = fb->current_vx;
        cfb.distance_remaining = total_dist - fb->current_distance;
        cfb.waypoint_id = seg.waypoint_to;
        on_feedback_(cfb);
      }
    };

  auto goal_future = translate_client_->async_send_goal(goal, send_goal_options);

  auto deadline = node_->now() + rclcpp::Duration::from_seconds(action_timeout_sec_);
  while (!segment_done_.load() && rclcpp::ok()) {
    if (state_.load() == ChainerState::CANCELLED) {
      translate_client_->async_cancel_all_goals();
      return;
    }
    if (node_->now() > deadline) {
      RCLCPP_WARN(node_->get_logger(),
        "[ActionChainer] Translate timeout for segment %u", seg.segment_id);
      translate_client_->async_cancel_all_goals();
      onSegmentComplete(-3);
      return;
    }
    std::this_thread::sleep_for(10ms);
  }

  onSegmentComplete(segment_status_, segment_distance_);
}

// ---------------------------------------------------------------------------
// executeTranslateReverse()
// ---------------------------------------------------------------------------
void ActionChainer::executeTranslateReverse(const waypoint_interfaces::msg::Segment & seg)
{
  if (!translate_reverse_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node_->get_logger(),
      "[ActionChainer] TranslateReverse server not available");
    onSegmentComplete(-3);
    return;
  }

  auto goal = TranslateAction::Goal();
  goal.start_x = seg.start_x;
  goal.start_y = seg.start_y;
  goal.end_x = seg.end_x;
  goal.end_y = seg.end_y;
  goal.max_linear_speed = -std::abs(seg.max_linear_speed);  // negative for reverse
  goal.acceleration = seg.acceleration;
  goal.exit_speed = seg.exit_speed;
  goal.has_next = seg.has_next;
  goal.control_mode = seg.control_mode;
  goal.hold_steer = seg.hold_steer;
  goal.exit_steer_angle = seg.exit_steer_angle;

  segment_done_.store(false);
  segment_status_ = 0;
  segment_distance_ = 0.0;

  auto send_goal_options = rclcpp_action::Client<TranslateAction>::SendGoalOptions();

  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<TranslateAction>::WrappedResult & wrapped) {
      std::lock_guard<std::mutex> lock(result_mutex_);
      segment_status_ = wrapped.result->status;
      segment_distance_ = wrapped.result->actual_distance;
      segment_done_.store(true);
    };

  send_goal_options.feedback_callback =
    [this, &seg](
      rclcpp_action::ClientGoalHandle<TranslateAction>::SharedPtr,
      const std::shared_ptr<const TranslateAction::Feedback> fb) {
      if (on_feedback_) {
        double total_dist = std::hypot(seg.end_x - seg.start_x, seg.end_y - seg.start_y);
        ChainFeedback cfb;
        cfb.current_segment_id = seg.segment_id;
        cfb.segment_action_type = seg.action_type;
        cfb.segment_phase = fb->phase;
        cfb.current_speed = fb->current_vx;
        cfb.distance_remaining = total_dist - std::abs(fb->current_distance);
        cfb.waypoint_id = seg.waypoint_to;
        on_feedback_(cfb);
      }
    };

  auto goal_future = translate_reverse_client_->async_send_goal(goal, send_goal_options);

  auto deadline = node_->now() + rclcpp::Duration::from_seconds(action_timeout_sec_);
  while (!segment_done_.load() && rclcpp::ok()) {
    if (state_.load() == ChainerState::CANCELLED) {
      translate_reverse_client_->async_cancel_all_goals();
      return;
    }
    if (node_->now() > deadline) {
      RCLCPP_WARN(node_->get_logger(),
        "[ActionChainer] TranslateReverse timeout for segment %u", seg.segment_id);
      translate_reverse_client_->async_cancel_all_goals();
      onSegmentComplete(-3);
      return;
    }
    std::this_thread::sleep_for(10ms);
  }

  onSegmentComplete(segment_status_, segment_distance_);
}

// ---------------------------------------------------------------------------
// executeTurn()
// ---------------------------------------------------------------------------
void ActionChainer::executeTurn(const waypoint_interfaces::msg::Segment & seg)
{
  if (!turn_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node_->get_logger(), "[ActionChainer] Turn server not available");
    onSegmentComplete(-3);
    return;
  }

  auto goal = TurnAction::Goal();
  goal.target_angle = seg.turn_angle;
  goal.turn_radius = seg.turn_radius;
  goal.max_linear_speed = seg.max_linear_speed;
  goal.accel_angle = turn_accel_angle_;  // from params, NOT 0.0
  goal.hold_steer = seg.hold_steer;

  segment_done_.store(false);
  segment_status_ = 0;
  segment_distance_ = 0.0;

  auto send_goal_options = rclcpp_action::Client<TurnAction>::SendGoalOptions();

  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<TurnAction>::WrappedResult & wrapped) {
      std::lock_guard<std::mutex> lock(result_mutex_);
      segment_status_ = wrapped.result->status;
      segment_done_.store(true);
    };

  send_goal_options.feedback_callback =
    [this, &seg](
      rclcpp_action::ClientGoalHandle<TurnAction>::SharedPtr,
      const std::shared_ptr<const TurnAction::Feedback> fb) {
      if (on_feedback_) {
        ChainFeedback cfb;
        cfb.current_segment_id = seg.segment_id;
        cfb.segment_action_type = seg.action_type;
        cfb.segment_phase = fb->phase;
        cfb.current_speed = fb->current_linear_speed;
        cfb.distance_remaining = fb->remaining_angle;
        cfb.waypoint_id = seg.waypoint_to;
        on_feedback_(cfb);
      }
    };

  auto goal_future = turn_client_->async_send_goal(goal, send_goal_options);

  auto deadline = node_->now() + rclcpp::Duration::from_seconds(action_timeout_sec_);
  while (!segment_done_.load() && rclcpp::ok()) {
    if (state_.load() == ChainerState::CANCELLED) {
      turn_client_->async_cancel_all_goals();
      return;
    }
    if (node_->now() > deadline) {
      RCLCPP_WARN(node_->get_logger(),
        "[ActionChainer] Turn timeout for segment %u", seg.segment_id);
      turn_client_->async_cancel_all_goals();
      onSegmentComplete(-3);
      return;
    }
    std::this_thread::sleep_for(10ms);
  }

  onSegmentComplete(segment_status_);
}

// ---------------------------------------------------------------------------
// executeYawControl()
// ---------------------------------------------------------------------------
void ActionChainer::executeYawControl(const waypoint_interfaces::msg::Segment & seg)
{
  if (!yaw_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node_->get_logger(), "[ActionChainer] YawControl server not available");
    onSegmentComplete(-3);
    return;
  }

  auto goal = YawControlAction::Goal();
  double dx = seg.end_x - seg.start_x;
  double dy = seg.end_y - seg.start_y;
  goal.target_heading = std::atan2(dy, dx) * 180.0 / M_PI;
  goal.max_linear_speed = seg.max_linear_speed;
  goal.acceleration = seg.acceleration;
  goal.deceleration = seg.acceleration;
  goal.stop_distance = std::hypot(dx, dy);
  goal.hold_steer = seg.hold_steer;

  segment_done_.store(false);
  segment_status_ = 0;
  segment_distance_ = 0.0;

  auto send_goal_options = rclcpp_action::Client<YawControlAction>::SendGoalOptions();

  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<YawControlAction>::WrappedResult & wrapped) {
      std::lock_guard<std::mutex> lock(result_mutex_);
      segment_status_ = wrapped.result->status;
      segment_distance_ = wrapped.result->total_distance;
      segment_done_.store(true);
    };

  send_goal_options.feedback_callback =
    [this, &seg](
      rclcpp_action::ClientGoalHandle<YawControlAction>::SharedPtr,
      const std::shared_ptr<const YawControlAction::Feedback> fb) {
      if (on_feedback_) {
        double total_dist = std::hypot(seg.end_x - seg.start_x, seg.end_y - seg.start_y);
        ChainFeedback cfb;
        cfb.current_segment_id = seg.segment_id;
        cfb.segment_action_type = seg.action_type;
        cfb.segment_phase = fb->phase;
        cfb.current_speed = fb->current_vx;
        cfb.distance_remaining = total_dist;  // YawControl feedback has no current_distance
        cfb.waypoint_id = seg.waypoint_to;
        on_feedback_(cfb);
      }
    };

  auto goal_future = yaw_client_->async_send_goal(goal, send_goal_options);

  auto deadline = node_->now() + rclcpp::Duration::from_seconds(action_timeout_sec_);
  while (!segment_done_.load() && rclcpp::ok()) {
    if (state_.load() == ChainerState::CANCELLED) {
      yaw_client_->async_cancel_all_goals();
      return;
    }
    if (node_->now() > deadline) {
      RCLCPP_WARN(node_->get_logger(),
        "[ActionChainer] YawControl timeout for segment %u", seg.segment_id);
      yaw_client_->async_cancel_all_goals();
      onSegmentComplete(-3);
      return;
    }
    std::this_thread::sleep_for(10ms);
  }

  onSegmentComplete(segment_status_, segment_distance_);
}

// ---------------------------------------------------------------------------
// executeWait() — ROS timer, NOT wall timer (sim_time safe)
// ---------------------------------------------------------------------------
void ActionChainer::executeWait(const waypoint_interfaces::msg::Segment & seg)
{
  double duration = seg.wait_duration;
  if (duration <= 0.0) { duration = 1.0; }

  RCLCPP_INFO(node_->get_logger(),
    "[ActionChainer] WAIT segment %u: %.1f sec", seg.segment_id, duration);

  segment_done_.store(false);
  segment_status_ = 0;

  // Use ROS timer (sim_time compatible), not wall timer
  wait_timer_ = rclcpp::create_timer(
    node_, node_->get_clock(),
    rclcpp::Duration::from_seconds(duration),
    [this]() {
      wait_timer_->cancel();
      wait_timer_.reset();
      std::lock_guard<std::mutex> lock(result_mutex_);
      segment_status_ = 0;
      segment_done_.store(true);
    });

  // Poll for completion on detached thread
  auto deadline = node_->now() + rclcpp::Duration::from_seconds(duration + 5.0);
  while (!segment_done_.load() && rclcpp::ok()) {
    if (state_.load() == ChainerState::CANCELLED) {
      if (wait_timer_) {
        wait_timer_->cancel();
        wait_timer_.reset();
      }
      return;
    }
    if (node_->now() > deadline) {
      RCLCPP_WARN(node_->get_logger(),
        "[ActionChainer] Wait timeout for segment %u", seg.segment_id);
      if (wait_timer_) {
        wait_timer_->cancel();
        wait_timer_.reset();
      }
      onSegmentComplete(-3);
      return;
    }
    std::this_thread::sleep_for(10ms);
  }

  onSegmentComplete(segment_status_);
}

// ---------------------------------------------------------------------------
// onSegmentComplete()
// ---------------------------------------------------------------------------
void ActionChainer::onSegmentComplete(int8_t status, double distance)
{
  if (status == 0) {
    completed_count_++;
    total_distance_ += distance;
    current_index_++;
    state_.store(ChainerState::EXECUTING);
    executeNext();
  } else {
    // Segment failed — report to caller
    state_.store(ChainerState::MISSION_FAILED);
    ChainResult result;
    result.status = status;
    result.completed_segments = completed_count_;
    result.total_distance = total_distance_;
    result.elapsed_time = (node_->now() - start_time_).seconds();
    if (on_complete_) { on_complete_(result); }
  }
}

}  // namespace waypoint_manager
