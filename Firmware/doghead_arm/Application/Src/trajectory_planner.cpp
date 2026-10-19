#include "trajectory_planner.h"
#include "robot_config.h"
#include <cmath>

JointPvaInterpolator g_pva_interpolator;

JointPvaInterpolator::JointPvaInterpolator() {
    reset();
}

void JointPvaInterpolator::reset() {
    for (int i = 0; i < 3; ++i) {
        _state.pos_rad[i] = 0.0f;
        _state.vel_rad_s[i] = 0.0f;
        _state.acc_rad_s2[i] = 0.0f;
    }
    _state.active = false;
}

void JointPvaInterpolator::init(float p0_rad, float p1_rad, float p2_rad) {
    _state.pos_rad[0] = p0_rad;
    _state.pos_rad[1] = p1_rad;
    _state.pos_rad[2] = p2_rad;
    _state.vel_rad_s[0] = 0.0f;
    _state.vel_rad_s[1] = 0.0f;
    _state.vel_rad_s[2] = 0.0f;
    _state.acc_rad_s2[0] = 0.0f;
    _state.acc_rad_s2[1] = 0.0f;
    _state.acc_rad_s2[2] = 0.0f;
    _state.active = true;
}

bool JointPvaInterpolator::update(const JointPvaPayload_t& target_pva, float dt_seconds, JointPvaState& out_state) {
    if (dt_seconds <= 0.0f || !std::isfinite(dt_seconds)) {
        reset();
        out_state = _state;
        return false;
    }

    if (!_state.active) {
        init(_state.pos_rad[0], _state.pos_rad[1], _state.pos_rad[2]);
    }

    static const float min_pos[3] = { JOINT1_MIN_RAD, JOINT2_MIN_RAD, JOINT3_MIN_RAD };
    static const float max_pos[3] = { JOINT1_MAX_RAD, JOINT2_MAX_RAD, JOINT3_MAX_RAD };

    for (int i = 0; i < 3; ++i) {
        float targ_p = target_pva.position_rad[i];
        float targ_v = target_pva.velocity_rad_s[i];
        float targ_a = target_pva.acceleration_rad_s2[i];

        if (!std::isfinite(targ_p) || !std::isfinite(targ_v) || !std::isfinite(targ_a)) {
            reset();
            out_state = _state;
            return false;
        }

        // Clamp target setpoints to safety limits
        if (targ_p < min_pos[i]) targ_p = min_pos[i];
        if (targ_p > max_pos[i]) targ_p = max_pos[i];
        if (targ_v > JOINT_PVA_MAX_VEL_RAD_S) targ_v = JOINT_PVA_MAX_VEL_RAD_S;
        if (targ_v < -JOINT_PVA_MAX_VEL_RAD_S) targ_v = -JOINT_PVA_MAX_VEL_RAD_S;
        if (targ_a > JOINT_PVA_MAX_ACCEL_RAD_S2) targ_a = JOINT_PVA_MAX_ACCEL_RAD_S2;
        if (targ_a < -JOINT_PVA_MAX_ACCEL_RAD_S2) targ_a = -JOINT_PVA_MAX_ACCEL_RAD_S2;

        const float curr_p = _state.pos_rad[i];
        const float curr_v = _state.vel_rad_s[i];
        const float position_error = targ_p - curr_p;
        const float error_abs = std::fabs(position_error);

        // Position is the authoritative PVA target.  A zero target velocity
        // is a terminal condition, not a request to remain at the current
        // position.  The braking-speed term produces a bounded velocity that
        // can still converge to the target without overshoot.
        float position_speed = std::sqrt(2.0f * JOINT_PVA_MAX_ACCEL_RAD_S2 * error_abs);
        if (position_speed > JOINT_PVA_MAX_VEL_RAD_S) {
            position_speed = JOINT_PVA_MAX_VEL_RAD_S;
        }
        float position_velocity = 0.0f;
        if (error_abs > 0.000001f) {
            position_velocity = (position_error > 0.0f) ? position_speed : -position_speed;
        } else {
            // At the position target, honour a finite terminal velocity only
            // when explicitly requested; normal endpoint samples use zero.
            position_velocity = targ_v;
        }

        // The incoming velocity is a feed-forward/terminal reference.  It may
        // not suppress the position correction required for convergence.
        if (error_abs > 0.000001f &&
            ((position_velocity > 0.0f && targ_v > position_velocity) ||
             (position_velocity < 0.0f && targ_v < position_velocity))) {
            position_velocity = targ_v;
        }

        float requested_acc = (position_velocity - curr_v) / dt_seconds;
        if (!std::isfinite(requested_acc)) {
            reset();
            out_state = _state;
            return false;
        }
        if (requested_acc > JOINT_PVA_MAX_ACCEL_RAD_S2) requested_acc = JOINT_PVA_MAX_ACCEL_RAD_S2;
        if (requested_acc < -JOINT_PVA_MAX_ACCEL_RAD_S2) requested_acc = -JOINT_PVA_MAX_ACCEL_RAD_S2;

        // Keep the position controller authoritative.  A target acceleration
        // may add feed-forward only when it reinforces the correction already
        // required by the position error; it can never cancel or reverse that
        // correction and strand the interpolator away from its target.
        float step_a = requested_acc;
        if (std::fabs(targ_a) > 0.0f &&
            (std::fabs(position_error) <= 0.000001f ||
             requested_acc == 0.0f ||
             ((requested_acc > 0.0f) == (targ_a > 0.0f)))) {
            step_a += 0.25f * targ_a;
        }
        if (step_a > JOINT_PVA_MAX_ACCEL_RAD_S2) step_a = JOINT_PVA_MAX_ACCEL_RAD_S2;
        if (step_a < -JOINT_PVA_MAX_ACCEL_RAD_S2) step_a = -JOINT_PVA_MAX_ACCEL_RAD_S2;

        float next_v = curr_v + step_a * dt_seconds;
        if (next_v > JOINT_PVA_MAX_VEL_RAD_S) next_v = JOINT_PVA_MAX_VEL_RAD_S;
        if (next_v < -JOINT_PVA_MAX_VEL_RAD_S) next_v = -JOINT_PVA_MAX_VEL_RAD_S;

        _state.acc_rad_s2[i] = step_a;

        float next_p = curr_p + curr_v * dt_seconds + 0.5f * step_a * dt_seconds * dt_seconds;

        if (next_p < min_pos[i]) {
            next_p = min_pos[i];
            next_v = 0.0f;
            _state.acc_rad_s2[i] = 0.0f;
        } else if (next_p > max_pos[i]) {
            next_p = max_pos[i];
            next_v = 0.0f;
            _state.acc_rad_s2[i] = 0.0f;
        }

        // Snap only after the generated trajectory actually reaches the
        // target.  This is the explicit completion state used by telemetry;
        // it is not an endpoint assignment on command receipt.
        if (error_abs <= 0.0005f && std::fabs(next_v) <= 0.002f) {
            next_p = targ_p;
            next_v = targ_v;
            if (std::fabs(next_v) <= 0.002f) {
                next_v = 0.0f;
                _state.acc_rad_s2[i] = 0.0f;
            }
        } else if ((position_error > 0.0f && next_p >= targ_p) ||
                   (position_error < 0.0f && next_p <= targ_p)) {
            next_p = targ_p;
            next_v = (std::fabs(targ_v) <= 0.002f) ? 0.0f : targ_v;
            _state.acc_rad_s2[i] = 0.0f;
        }

        _state.pos_rad[i] = next_p;
        _state.vel_rad_s[i] = next_v;
    }

    out_state = _state;
    return true;
}
