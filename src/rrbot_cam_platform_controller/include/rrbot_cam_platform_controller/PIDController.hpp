#ifndef RRBOT_CAM_OBJECT_FOLLOWER__PIDCONTROLLER_HPP_
#define RRBOT_CAM_OBJECT_FOLLOWER__PIDCONTROLLER_HPP_

#include <cmath>

namespace rrbot_cam_platform_controller
{

class PIDController
{

public:
    PIDController(double KP, double KI, double KD, double min_ref, double max_ref, double min_output, double max_output);
    ~PIDController() = default;

    void set_gains(double KP, double KI, double KD);
    double compute(double ref);

private:
    double K_P_, K_I_, K_D_;
    
    double min_ref_, max_ref_;
    double min_output_, max_output_;
    double prev_error_, integral_error_;

};

} // namespace rrbot_cam_object_follower

#endif // RRBOT_CAM_OBJECT_FOLLOWER__PIDCONTROLLER_HPP_