#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#ifdef __cplusplus
#include <cmath>
#include "robot_interface.h"

struct JointPvaState {
    float pos_rad[3];
    float vel_rad_s[3];
    float acc_rad_s2[3];
    bool active;
};

class JointPvaInterpolator {
public:
    JointPvaInterpolator();

    void reset();
    void init(float p0_rad, float p1_rad, float p2_rad);
    bool update(const JointPvaPayload_t& target_pva, float dt_seconds, JointPvaState& out_state);

    const JointPvaState& getState() const { return _state; }
    bool isActive() const { return _state.active; }

private:
    JointPvaState _state;
};

extern JointPvaInterpolator g_pva_interpolator;

#endif // __cplusplus

#endif // TRAJECTORY_PLANNER_H

