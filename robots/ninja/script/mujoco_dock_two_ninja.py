#!/usr/bin/env python3

import argparse
import math
import sys
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


def call_service(service_name, description):
    rospy.loginfo("Waiting for %s", service_name)
    rospy.wait_for_service(service_name, timeout=10.0)
    response = rospy.ServiceProxy(service_name, Trigger)()
    if not response.success:
        rospy.logerr("%s failed: %s", description, response.message)
        return False
    rospy.loginfo("%s: %s", description, response.message)
    return True


class MotionMonitor:
    def __init__(self):
        self._lock = threading.Lock()
        self._messages = {}
        self._subscribers = [
            rospy.Subscriber(
                "/ninja1/ground_truth", Odometry, self._callback,
                callback_args="ninja1", queue_size=1),
            rospy.Subscriber(
                "/ninja2/ground_truth", Odometry, self._callback,
                callback_args="ninja2", queue_size=1),
        ]

    def _callback(self, message, name):
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        with self._lock:
            self._messages[name] = (
                math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2),
                math.sqrt(angular.x ** 2 + angular.y ** 2 + angular.z ** 2),
                time.monotonic())

    def wait_until_stationary(self, stable_duration, timeout,
                              max_linear_speed, max_angular_speed):
        if stable_duration <= 0.0:
            return True

        rospy.loginfo(
            "Waiting until both modules are stationary for %.2f s",
            stable_duration)
        deadline = time.monotonic() + timeout
        stable_since = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            now = time.monotonic()
            with self._lock:
                samples = list(self._messages.values())
            stationary = (
                len(samples) == 2 and
                all(now - sample[2] < 0.25 for sample in samples) and
                all(sample[0] <= max_linear_speed and
                    sample[1] <= max_angular_speed for sample in samples))
            if stationary:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= stable_duration:
                    rospy.loginfo("Both modules are stationary")
                    return True
            else:
                stable_since = None
            time.sleep(0.02)

        rospy.logerr(
            "Modules did not become stationary within %.1f s; refusing to "
            "teleport and weld moving bodies", timeout)
        return False


def main():
    parser = argparse.ArgumentParser(
        description=("Align ninja2/pitch_connect_point with "
                     "ninja1/yaw_connect_point and weld the two modules."))
    parser.add_argument(
        "--detach", action="store_true",
        help="release the MuJoCo weld instead of aligning and attaching")
    parser.add_argument(
        "--settle-time", type=float, default=10.0,
        help=("simulation seconds to let the state estimator settle after the "
              "instantaneous alignment (default: 10)"))
    parser.add_argument(
        "--pre-settle-time", type=float, default=0.5,
        help=("continuous stationary time required before attachment "
              "(default: 0.5; use 0 to disable)"))
    parser.add_argument(
        "--stationary-timeout", type=float, default=15.0,
        help="wall-clock timeout for the stationary check (default: 15)")
    parser.add_argument(
        "--max-linear-speed", type=float, default=0.02,
        help="stationary linear-speed threshold in m/s (default: 0.02)")
    parser.add_argument(
        "--max-angular-speed", type=float, default=0.05,
        help="stationary angular-speed threshold in rad/s (default: 0.05)")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("mujoco_dock_two_ninja", anonymous=True)
    if args.detach:
        success = call_service("/mujoco/docking/detach", "Detach")
    else:
        monitor = MotionMonitor()
        success = monitor.wait_until_stationary(
            args.pre_settle_time, args.stationary_timeout,
            args.max_linear_speed, args.max_angular_speed)
        # Alignment and weld activation are performed atomically in one
        # MuJoCo update, preventing fast headless simulation from moving the
        # modules apart between the two operations.
        if success:
            success = call_service(
                "/mujoco/docking/align_and_attach", "Align and attach")
        if success and args.settle_time > 0.0:
            rospy.loginfo(
                "Waiting %.1f simulation seconds for estimators to settle",
                args.settle_time)
            rospy.sleep(args.settle_time)
            rospy.loginfo("Docking settle period complete")
            # Do not invoke align_and_attach again here.  A compliant weld can
            # retain a small constraint displacement under gravity; treating
            # that as a new alignment request teleports the follower while
            # estimators are running and can create a large state transient.
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
