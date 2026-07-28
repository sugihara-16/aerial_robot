#!/usr/bin/env python3

"""Publish one ordered six-axis interaction-wrench target reliably."""

import argparse
import time

import rospy
from beetle.msg import TaggedWrench


def parse_args():
    parser = argparse.ArgumentParser(
        description="Set one Ninja contact-wrench target using a stamped command")
    parser.add_argument("contact_id", type=int,
                        help="left module/contact index (1 means ninja1--ninja2)")
    parser.add_argument("fx", type=float)
    parser.add_argument("fy", type=float)
    parser.add_argument("fz", type=float)
    parser.add_argument("tx", type=float)
    parser.add_argument("ty", type=float)
    parser.add_argument("tz", type=float)
    parser.add_argument(
        "--topic", default=None,
        help="input topic (default: /ninja<contact_id>/ff_inter_wrench)")
    parser.add_argument(
        "--min-subscribers", type=int, default=2,
        help="required controller subscribers before publishing (default: 2)")
    parser.add_argument(
        "--connection-timeout", type=float, default=5.0,
        help="seconds to wait for all controller subscribers")
    parser.add_argument(
        "--publish-duration", type=float, default=0.5,
        help="seconds to repeat this command with the same ordering stamp")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="repeat rate in Hz")
    return parser.parse_args(rospy.myargv()[1:])


def wall_time_to_ros_time(wall_time_ns):
    seconds, nanoseconds = divmod(wall_time_ns, 1_000_000_000)
    return rospy.Time(seconds, nanoseconds)


def main():
    # Capture ordering before ROS connection setup.  If an older process takes
    # longer to connect, its original (older) stamp is retained and controllers
    # can reject it after a newer command has already arrived.
    command_wall_time_ns = time.time_ns()
    rospy.init_node("ff_inter_wrench_cmd", anonymous=True)
    args = parse_args()

    if args.contact_id <= 0:
        raise ValueError("contact_id must be positive")
    if args.min_subscribers <= 0:
        raise ValueError("min-subscribers must be positive")
    if args.rate <= 0.0 or args.publish_duration <= 0.0:
        raise ValueError("rate and publish-duration must be positive")

    topic = args.topic or "/ninja{}/ff_inter_wrench".format(args.contact_id)
    publisher = rospy.Publisher(topic, TaggedWrench, queue_size=1, latch=False)

    deadline = time.monotonic() + args.connection_timeout
    while (not rospy.is_shutdown()
           and publisher.get_num_connections() < args.min_subscribers
           and time.monotonic() < deadline):
        rospy.sleep(0.02)

    subscribers = publisher.get_num_connections()
    if subscribers < args.min_subscribers:
        rospy.logerr("%s has %d subscribers; %d required. Command was not sent.",
                     topic, subscribers, args.min_subscribers)
        return 2

    msg = TaggedWrench()
    msg.index = args.contact_id
    msg.wrench.header.stamp = wall_time_to_ros_time(command_wall_time_ns)
    msg.wrench.header.seq = command_wall_time_ns & 0xffffffff
    msg.wrench.header.frame_id = "ordered_wall_time"
    msg.wrench.wrench.force.x = args.fx
    msg.wrench.wrench.force.y = args.fy
    msg.wrench.wrench.force.z = args.fz
    msg.wrench.wrench.torque.x = args.tx
    msg.wrench.wrench.torque.y = args.ty
    msg.wrench.wrench.torque.z = args.tz

    rospy.loginfo("Publishing contact %d target on %s: [%g %g %g %g %g %g]",
                  args.contact_id, topic, args.fx, args.fy, args.fz,
                  args.tx, args.ty, args.tz)
    rate = rospy.Rate(args.rate)
    end_time = time.monotonic() + args.publish_duration
    while not rospy.is_shutdown() and time.monotonic() < end_time:
        publisher.publish(msg)
        rate.sleep()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
