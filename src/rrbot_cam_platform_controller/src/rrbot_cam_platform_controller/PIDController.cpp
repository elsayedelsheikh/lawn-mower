#include <algorithm>
#include "rrbot_cam_platform_controller/PIDController.hpp"

namespace rrbot_cam_platform_controller
{

PIDController::PIDController(double KP, double KI, double KD, double min_ref, double max_ref, double min_output, double max_output):
    K_P_(KP),
    K_I_(KI),
    K_D_(KD),
    min_ref_(min_ref),
    max_ref_(max_ref),
    min_output_(min_output),
    max_output_(max_output),
    prev_error_(0.0),
    integral_error_(0.0)
{}

void
PIDController::set_gains(double KP, double KI, double KD)
{
    K_P_ = KP;
    K_I_ = KI;
    K_D_ = KD;
}

double
PIDController::compute(double ref)
{
    double output = 0.0;

    // Proportional term
    double direction = 0.0;
    if (ref != 0.0)
        direction = ref / fabs(ref);

    if (fabs(ref) < min_ref_){
        output = 0.0;
    } else if (fabs(ref) > max_ref_){
        output = direction * max_output_;
    } else {
        output = direction * min_output_ + ref * (max_output_ - min_output_);
    }

    // Integral term
    integral_error_ = (integral_error_ + output) * 2.0 / 3.0;

    // Derivative term
    double derivative_error = output - prev_error_;
    prev_error_ = output;

    output = K_P_ * output + K_I_ * integral_error_ + K_D_ * derivative_error;

    return std::clamp(output, min_output_, max_output_);
}

} // namespace rrbot_cam_object_follower