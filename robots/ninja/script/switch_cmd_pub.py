#!/usr/bin/env python

import rospy
import time
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import Bool
from ninja.kondo_control_api import KondoControl


def get_switch_param(name, default):
    """Accept both conventional private params and the legacy launch globals."""
    if rospy.has_param("~" + name):
        return rospy.get_param("~" + name)
    return rospy.get_param(name, default)


if __name__=="__main__":
    rospy.init_node("switch_cmd_pub")
    real_machine = get_switch_param("real_machine", False)
    switch_type = get_switch_param("switch_type", 1) # 1 -> assembly, 0/2 -> disassembly
    assembly = switch_type == 1

    male_servo_id = get_switch_param("male_servo_id", 5)
    female_servo_id = get_switch_param("female_servo_id", 6)
    unlock_servo_angle_male = get_switch_param("unlock_servo_angle_male", 7000)
    lock_servo_angle_male = get_switch_param("lock_servo_angle_male", 8300)
    unlock_servo_angle_female = get_switch_param("unlock_servo_angle_female", 11000)
    lock_servo_angle_female = get_switch_param("lock_servo_angle_female", 5600)

    left_edge_id = get_switch_param("left_edge_id", 1)
    right_edge_id = get_switch_param("right_edge_id", 3)
    
    robot_ids = list(range(left_edge_id, right_edge_id + 1))
    rospy.loginfo("Switching modules %s to %s", robot_ids,
                  "assembled" if assembly else "separated")
    flag_pubs = []
    docking_pubs =[]
    male_servo_handlers = []
    female_servo_handlers = []
    
    flag_msg = KeyValue()
    docking_msg = Bool()

    for index, id in enumerate(robot_ids):
        robot_name = 'ninja' + str(id)
        # Assembly is state, not an edge-triggered command.  Latching plus a
        # short repeated publish prevents one module from missing the state and
        # entering a controller mode inconsistent with its welded neighbour.
        flag_pubs.append(rospy.Publisher('/' +robot_name  + '/assembly_flag',
                                         KeyValue, queue_size=1, latch=True))
        docking_pubs.append(rospy.Publisher('/' +robot_name  + '/docking_cmd',
                                            Bool, queue_size=1, latch=True))
        male_servo_handlers.append(KondoControl(robot_name,id,male_servo_id,real_machine))
        female_servo_handlers.append(KondoControl(robot_name,id,female_servo_id,real_machine))

    # Every running module subscribes to every assembly flag in the chain.
    # Wait for those TCPROS links instead of relying on publisher construction
    # timing.  Continue after the timeout so real-machine partial deployments
    # can still be operated, but report the missing links explicitly.
    connection_deadline = time.monotonic() + 5.0
    expected_connections = len(robot_ids)
    missing = list(robot_ids)
    while not rospy.is_shutdown():
        missing = [robot_ids[i] for i, pub in enumerate(flag_pubs)
                   if pub.get_num_connections() < expected_connections]
        if not missing or time.monotonic() >= connection_deadline:
            break
        rospy.sleep(0.05)
    if missing:
        rospy.logwarn("Assembly flag subscribers were not all connected for modules %s", missing)

    flag_msg.value = '1' if assembly else '0'
    docking_msg.data = assembly
    for index, id in enumerate(robot_ids):
        flag_msg.key = str(id)
        flag_pubs[index].publish(flag_msg)
        if real_machine:
            if id == left_edge_id:
                if assembly:
                    male_servo_handlers[index].sendTargetAngle(lock_servo_angle_male)
                else:
                    male_servo_handlers[index].sendTargetAngle(unlock_servo_angle_male)
            elif id == right_edge_id:
                if assembly:
                    female_servo_handlers[index].sendTargetAngle(lock_servo_angle_female)
                else:
                    female_servo_handlers[index].sendTargetAngle(unlock_servo_angle_female)
            else:
                if assembly:
                    female_servo_handlers[index].sendTargetAngle(lock_servo_angle_female)
                    time.sleep(1.0)
                    male_servo_handlers[index].sendTargetAngle(lock_servo_angle_male)
                else:
                    female_servo_handlers[index].sendTargetAngle(unlock_servo_angle_female)
                    time.sleep(1.0)
                    male_servo_handlers[index].sendTargetAngle(unlock_servo_angle_male)
        else:
            docking_pubs[index].publish(docking_msg)

    # Keep the process alive briefly and repeat the state for transports that
    # were still completing their handshake at the first publication.
    for unused in range(10):
        if rospy.is_shutdown():
            break
        for index, id in enumerate(robot_ids):
            flag_msg.key = str(id)
            flag_pubs[index].publish(flag_msg)
            if not real_machine:
                docking_pubs[index].publish(docking_msg)
        rospy.sleep(0.1)
