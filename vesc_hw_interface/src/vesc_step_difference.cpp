/*********************************************************************
 * Copyright (c) 2023 SoftBank Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ********************************************************************/

#include "vesc_hw_interface/vesc_step_difference.h"
namespace vesc_step_difference
{
/**
 * Hall sensors attached to brushless motors have low angular resolution.
 * When looking at the time variation of the counter value obtained from the Hall sensor,
 * the value is slow and oscillatory. (The value will be either 0 or 1)
 * To prevent this, when the speed is slow, the sampling time is increased and the time variation of the counter value
 * is smoothed. This means that the smoothing filter is applied only at low speeds.
 */
VescStepDifference::VescStepDifference()
{
}

VescStepDifference::~VescStepDifference()
{
}

/**
 * Enables smoothing and initializes internal parameters.
 * If this function is not called, smoothing is disabled and the getStepDifference function outputs the difference from
 * the previous step as is.
 */
void VescStepDifference::enableSmooth(double control_rate, double max_sampling_time, int max_step_diff)
{
  enable_smooth_ = true;
  step_diff_vw_max_step_ = max_step_diff;
  step_diff_vw_max_window_size_ = std::max(1, static_cast<int>(control_rate * max_sampling_time));
  step_input_queue_.resize(step_diff_vw_max_window_size_ + 1);
  return;
}

/**
 * Outputs the difference between the previous input and the current input.
 * Smoothing is performed when "enable_smooth_" is true
 */
double VescStepDifference::getStepDifference(const double step_in, bool reset)
{
  double output;
  if (enable_smooth_)
  {
    output = this->stepDifferenceVariableWindow(step_in, reset);
  }
  else
  {
    output = this->stepDifferenceRaw(step_in, reset);
  }
  return output;
}

/**
 * Function called when the filter is disabled
 *
 * Outputs the difference of the counter value from the previous value.
 * This function is used when no filter is used.
 *
 * @step_in Enter the current counter value
 * @reset Used for initialization.
 * @return Returns the difference from the previous value.
 */
double VescStepDifference::stepDifferenceRaw(const double step_in, bool reset)
{
  if (reset)
  {
    step_in_prev_ = static_cast<int16_t>(step_in);
    return 0.0;
  }
  int16_t step_diff = static_cast<int16_t>(step_in) - step_in_prev_;
  step_in_prev_ = static_cast<int16_t>(step_in);
  return static_cast<double>(step_diff);
}

/**
 * Function called when the filter is enabled
 *
 * Output smoothed differences.
 * The smaller the difference, the longer the sampling time is stretched and smoothed.
 * If the latest difference is greater than max_step_diff, no smoothing is performed.
 * Smoothing is performed for max_sampling_time seconds when the latest difference is 0.
 *
 * @step_in Enter the current counter value
 * @reset Used for initialization.
 * @return Returns the difference from the previous value.
 */
double VescStepDifference::stepDifferenceVariableWindow(const double step_in, bool reset)
{
  if (reset)
  {
    // Fill the buffer with the current value
    std::fill(step_input_queue_.begin(), step_input_queue_.end(), static_cast<int16_t>(step_in));
    return 0.0;
  }
  // Increment buffer
  step_input_queue_.pop_back();
  step_input_queue_.push_front(static_cast<int16_t>(step_in));
  // Calculate window size
  int latest_step_diff = std::abs(step_input_queue_[0] - step_input_queue_[1]);
  int window_size = step_diff_vw_max_window_size_;
  if (latest_step_diff >= step_diff_vw_max_step_)
  {
    window_size = 1;
  }
  else if (latest_step_diff > 0)
  {
    window_size = step_diff_vw_max_window_size_ *
                  (1.0 - static_cast<double>(latest_step_diff) / static_cast<double>(step_diff_vw_max_step_));
  }
  // Get output
  int16_t step_diff = step_input_queue_[0] - step_input_queue_[window_size];
  return static_cast<double>(step_diff) / static_cast<double>(window_size);
}

}  // namespace vesc_step_difference
