#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState

JOINT_NAMES = ['mod1/yaw', 'mod2/pitch']


def min_jerk(q0, q1, t, T):
    """
    初期・終端速度/加速度ゼロを仮定した minimum jerk 軌道のスカラー版
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

    # パラメータ取得（必要に応じて rosparam で上書き）
    # 例: rosparam set /minimum_jerk_joint_commander/target_positions "[1.0, 0.5]"
    target_positions = rospy.get_param('~target_positions', [1.4, 0.0])
    initial_positions = rospy.get_param('~initial_positions', [0.0, 0.0])
    duration = rospy.get_param('~duration', 10.0)          # [s]
    control_period = rospy.get_param('~control_period', 0.01)  # [s]
    required_subscribers = rospy.get_param('~required_subscribers', 2)

    if len(target_positions) != 2 or len(initial_positions) != 2:
        rospy.logerr("target_positions と initial_positions は長さ 2 のリストで指定してください。")
        return

    if duration <= 0.0:
        rospy.logwarn("duration が 0 以下です。1.0 秒に補正します。")
        duration = 1.0

    if control_period <= 0.0:
        rospy.logwarn("control_period が 0 以下です。0.01 秒に補正します。")
        control_period = 0.01

    if required_subscribers < 1:
        rospy.logwarn("required_subscribers が 1 未満です。1 に補正します。")
        required_subscribers = 1

    pub = rospy.Publisher("/assembly/target_joint_pos",
                          JointState, queue_size=10)

    # rospy.Time.now() is temporarily zero until the first /clock message when
    # /use_sim_time is enabled.  On a real machine, wall-clock ROS time is
    # already non-zero and this loop exits immediately.
    if rospy.Time.now().is_zero():
        rospy.loginfo("ROS 時刻の初期化を待っています。")
    while not rospy.is_shutdown() and rospy.Time.now().is_zero():
        rospy.sleep(0.01)

    if rospy.is_shutdown():
        return

    rospy.loginfo("/assembly/target_joint_pos の subscriber を待っています (%d required)。",
                  required_subscribers)
    while (not rospy.is_shutdown() and
           pub.get_num_connections() < required_subscribers):
        rospy.sleep(0.05)

    if rospy.is_shutdown():
        return

    rate = rospy.Rate(1.0 / control_period)
    start_time = rospy.Time.now()

    rospy.loginfo("Minimum jerk 軌道の publish を開始します。")
    rospy.loginfo("  initial_positions = %s", initial_positions)
    rospy.loginfo("  target_positions  = %s", target_positions)
    rospy.loginfo("  duration          = %.3f [s]", duration)
    rospy.loginfo("  control_period    = %.3f [s]", control_period)
    rospy.loginfo("  subscribers       = %d", pub.get_num_connections())

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t = (now - start_time).to_sec()

        # 2関節分の軌道生成
        q_cmd = []
        for i in range(2):
            q = min_jerk(initial_positions[i],
                         target_positions[i],
                         t,
                         duration)
            q_cmd.append(q)

        # JointState メッセージ作成
        msg = JointState()
        msg.header.stamp = now
        msg.name = JOINT_NAMES
        msg.position = q_cmd  # 位置のみ。速度・加速度は設定しない

        pub.publish(msg)

        # Publish the exact terminal position once, then exit.  Continuing to
        # publish forever makes it difficult to distinguish active motion from
        # a completed trajectory.
        if t >= duration:
            rospy.loginfo("Minimum jerk 軌道が完了しました。")
            break

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
