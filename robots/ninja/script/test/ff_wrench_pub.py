#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from beetle.msg import TaggedWrench
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3
from std_msgs.msg import Header

def publish_once():
    rospy.init_node('tagged_wrench_multi_publisher', anonymous=True)

    # ninja番号とトピックを対応づけ
    ninja_topics = {
        1: "/ninja1/ff_inter_wrench",
        2: "/ninja2/ff_inter_wrench",
        3: "/ninja3/ff_inter_wrench",
        # 4: "/ninja4/ff_inter_wrench"
    }

    # Publisher作成
    pubs = {i: rospy.Publisher(topic, TaggedWrench, queue_size=10) 
            for i, topic in ninja_topics.items()}

    rospy.sleep(0.5)  # Publisher接続待ち

    for i, topic in ninja_topics.items():
        # Header作成
        header = Header()
        # header.seq = 1
        # header.stamp = rospy.Time.now()
        # header.frame_id = "base_link"

        # Wrench作成
        force = Vector3(1.0, 1.0, 1.0)
        torque = Vector3(0.5, 0.5, 0.5)
        # force = Vector3(0, 0, 0)
        # torque = Vector3(0, 0, 0)
        wrench = Wrench(force=force, torque=torque)

        # WrenchStamped作成
        wrench_stamped = WrenchStamped(header=header, wrench=wrench)

        # TaggedWrench作成
        msg = TaggedWrench()
        msg.index = i   # ninja番号をindexに設定
        msg.wrench = wrench_stamped

        rospy.loginfo("Publishing to %s with index=%d" % (topic, i))
        pubs[i].publish(msg)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        publish_once()
    except rospy.ROSInterruptException:
        pass
