#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState

JOINT_NAMES = ['mod1/yaw', 'mod2/pitch']


def min_jerk(q0, q1, t, T):
    """
     q(t) = q0 + (q1 - q0) * [10 s^3 - 15 s^4 + 6 s^5], s = t/T
    """
    if t <= 0.0:
        return q0
    if t >= T:
        return q1

    s = t / T
    s3 = s * s * s
    s4 = s3 * s
    s5 = s4 * s
    blend = 10.0 * s3 - 15.0 * s4 + 6.0 * s5
    return q0 + (q1 - q0) * blend


def main():
    rospy.init_node('minimum_jerk_joint_commander')

    target_positions = rospy.get_param('~target_positions', [1.0, 0.8])
    initial_positions = rospy.get_param('~initial_positions', [0.0, 0.0])
    duration = rospy.get_param('~duration', 16.0)          # [s]
    control_period = rospy.get_param('~control_period', 0.01)  # [s]

    if len(target_positions) != 2 or len(initial_positions) != 2:
        return

    if duration <= 0.0:
        duration = 0.1

    pub = rospy.Publisher("/assembly/target_joint_pos",
                          JointState, queue_size=10)

    rate = rospy.Rate(1.0 / control_period)
    start_time = rospy.Time.now()

    rospy.loginfo("  initial_positions = %s", initial_positions)
    rospy.loginfo("  target_positions  = %s", target_positions)
    rospy.loginfo("  duration          = %.3f [s]", duration)
    rospy.loginfo("  control_period    = %.3f [s]", control_period)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t = (now - start_time).to_sec()

        q_cmd = []
        for i in range(2):
            q = min_jerk(initial_positions[i],
                         target_positions[i],
                         t,
                         duration)
            q_cmd.append(q)

        msg = JointState()
        msg.header.stamp = now
        msg.name = JOINT_NAMES
        msg.position = q_cmd

        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
