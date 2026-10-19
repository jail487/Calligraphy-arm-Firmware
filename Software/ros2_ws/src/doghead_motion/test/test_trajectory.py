import pytest
from doghead_motion.trajectory_generator import JointPvaTrajectoryGenerator


def test_trajectory_bounds():
    gen = JointPvaTrajectoryGenerator(max_velocity=[1.0, 1.0, 1.0], max_acceleration=[2.0, 2.0, 2.0])

    start = [0.0, 0.0, 0.0]
    target = [0.5, -0.3, 0.4]

    t_total = gen.compute_motion_time(start, target)
    assert t_total > 0.0

    # Sample at start (progress=0.0)
    pos_0, vel_0, acc_0 = gen.sample_trajectory(start, target, 0.0)
    assert pos_0 == start
    assert vel_0 == [0.0, 0.0, 0.0]

    # Sample at end (progress=1.0)
    pos_1, vel_1, acc_1 = gen.sample_trajectory(start, target, 1.0)
    for i in range(3):
        assert abs(pos_1[i] - target[i]) < 1e-4
        assert abs(vel_1[i]) < 1e-4


def test_position_target_stream_is_sampled_at_100_hz():
    gen = JointPvaTrajectoryGenerator(max_velocity=[1.0] * 3, max_acceleration=[2.0] * 3)
    start = [0.0, 0.0, 0.0]
    target = [0.5, 0.0, 0.0]
    samples = list(gen.sample_stream(start, target, sample_rate_hz=100.0))

    assert len(samples) > 2
    assert samples[0][0][0] == start[0]
    assert abs(samples[-1][0][0] - target[0]) < 1e-6
    assert any(sample[0][0] > start[0] for sample in samples[1:-1])


def test_zero_target_velocity_and_acceleration_still_converges():
    gen = JointPvaTrajectoryGenerator(max_velocity=[1.0] * 3, max_acceleration=[2.0] * 3)
    start = [0.0, 0.0, 0.0]
    target = [0.5, 0.0, 0.0]
    duration = gen.compute_motion_time(start, target)
    pos, vel, acc = gen.sample_trajectory_at_time(start, target, duration)
    assert abs(pos[0] - target[0]) < 1e-6
    assert abs(vel[0]) < 1e-6
    assert abs(acc[0]) < 1e-6
