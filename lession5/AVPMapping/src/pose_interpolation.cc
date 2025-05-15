#include "avp_mapping/pose_interpolation.h"

namespace {

TimedPose interpolation(const TimedPose& start, const TimedPose& end,
                        double time) {
  double factor = (time - start.time_) / (end.time_ - start.time_);
  TimedPose ret;
  ret.time_ = time;
  ret.t_ = start.t_ + factor * (end.t_ - start.t_);
  ret.R_ = start.R_.slerp(factor, end.R_);
  return ret;
}

}  // namespace

void PoseInterpolation::Push(const TimedPose& pose) {
  if (pose.time_ > EarliestTime()) {
    poses_.push_back(pose);
  }
}

void PoseInterpolation::TrimBefore(double time) {
  while (!poses_.empty() && poses_.front().time_ < time) {
    poses_.pop_front();
  }
}

double PoseInterpolation::EarliestTime() const {
  return poses_.empty() ? 0. : poses_.front().time_;
}

bool PoseInterpolation::LookUp(TimedPose& pose) const {
  if (poses_.empty() || pose.time_ < poses_.front().time_ ||
      pose.time_ > poses_.back().time_) {
    return false;
  }

  auto end = std::lower_bound(
      poses_.begin(), poses_.end(), pose.time_,
      [](const TimedPose& _pose, const double t) { return _pose.time_ < t; });

  if (end == poses_.end()) {
    end = std::prev(end);
  }
  pose = interpolation(*std::prev(end), *end, pose.time_);
  return true;
}
