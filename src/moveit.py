#!/usr/bin/python2
import rospy
from std_msgs.msg import *
import time
intial_pos = [0, 0, 0, 0, 0, 0, 0]
pos_1 = [0, 0.25, -0.3, 0, -0.25, 0, 0]
pos_2 = [0, 0.3, -0.3, 0, -0.3, 0, 0]


def pub_angle(t1, t2, t3, t4, t5, t6, t7):
    pub_r_shoulder_pan_joint_controller.publish(t1)
    pub_r_shoulder_lift_joint_controller.publish(t2)
    pub_r_elbow_joint_controller.publish(t3)
    pub_r_wrist_1_joint_controller.publish(t4)
    pub_r_wrist_2_joint_controller.publish(t5)
    pub_r_wrist_3_joint_controller.publish(t6)


def move():

    global pub_r_shoulder_pan_joint_controller, pub_r_shoulder_lift_joint_controller, pub_r_elbow_joint_controller, pub_r_wrist_1_joint_controller, pub_r_wrist_2_joint_controller, pub_r_wrist_3_joint_controller
    pub_r_shoulder_pan_joint_controller = rospy.Publisher(
        '/l_shoulder_pan_joint_controller/command', Float64, queue_size=1, latch=False)
    pub_r_shoulder_lift_joint_controller = rospy.Publisher(
        '/l_shoulder_lift_joint_controller/command', Float64, queue_size=1, latch=False)
    pub_r_elbow_joint_controller = rospy.Publisher(
        '/l_elbow_joint_controller/command', Float64, queue_size=1, latch=False)
    pub_r_wrist_1_joint_controller = rospy.Publisher(
        '/l_wrist_1_joint_controller/command', Float64, queue_size=1, latch=False)
    pub_r_wrist_2_joint_controller = rospy.Publisher(
        '/l_wrist_2_joint_controller/command', Float64, queue_size=1, latch=False)
    pub_r_wrist_3_joint_controller = rospy.Publisher(
        '/l_wrist_3_joint_controller/command', Float64, queue_size=1, latch=False)
    # global pub_r_shoulder_pan_joint_controller,pub_r_shoulder_lift_joint_controller,pub_r_elbow_joint_controller,pub_r_wrist_1_joint_controller,pub_r_wrist_2_joint_controller,pub_r_wrist_3_joint_controller
    rospy.init_node('move', anonymous=True)

    rate = rospy.Rate(100)  # 10hz
    pub_angle(
        intial_pos[0],
        intial_pos[1],
        intial_pos[2],
        intial_pos[3],
        intial_pos[4],
        intial_pos[5],
        intial_pos[6])
    time.sleep(2)
    pub_angle(
        pos_1[0],
        pos_1[1],
        pos_1[2],
        pos_1[3],
        pos_1[4],
        pos_1[5],
        pos_1[6])
    time.sleep(2)
    pub_angle(
        pos_2[0],
        pos_2[1],
        pos_2[2],
        pos_2[3],
        pos_2[4],
        pos_2[5],
        pos_2[6])
    time.sleep(2)


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
