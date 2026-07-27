#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#ifdef __cplusplus
#include <cmath>

class TrapezoidalPlanner {
public:
    TrapezoidalPlanner();

    void reset();
    void setTarget(float start_pos, float target_pos, float max_vel, float max_acc);
    void update(float target_pos_in, float dt);

    float getPosition() const { return _current_pos; }
    float getVelocity() const { return _current_vel; }
    float getAcceleration() const { return _last_acc; }

private:
    float _current_pos;
    float _current_vel;
    float _last_acc;
    
    float _target_pos;
    float _max_vel;
    float _max_acc;
};

// Joint planner C++ instances
extern TrapezoidalPlanner traj_joint1;
extern TrapezoidalPlanner traj_joint2;
extern TrapezoidalPlanner traj_joint3;

#endif // __cplusplus

#endif // TRAJECTORY_PLANNER_H
