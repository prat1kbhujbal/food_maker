#!/usr/bin/python2.7
import time
import rospy
import numpy as np
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelState


x = -1.5

# right arm joint angles
r_intial_pos = [0, 0, 0, 0, 0, 0, 0.01, 0.01]
r_pos_1 = [-1.3, 0, 0, 0, 0, 0, 0.01, 0.01]
r_pos_2 = [-1.35, 0, 0.4, 0, 0, 0, 0.5, 0.5]
r_pos_3 = [-1.4, 0.2, 0.4, 0, 0, 0, 0.5, 0.5]
r_pos_4 = [-1.4, 0.2, 0.4, 0, 0, 0, 0.7, 0.7]
r_pos_5 = [-1.7, 0.2, 0.4, 0, 0, 0, 0.7, 0.7]
r_pos_6 = [-1.7, 0.2, 0.4, 0, 0, 0, 0.01, 0.01]
r_pos_7 = [-1.4, 0.2, 0.4, 0, 0, 0, 0.01, 0.01]
r_pos_8 = [-1.35, 0.2, 0.4, 0, 0, 0, 0.0, 0.0]
r_pos_9 = [-1.35, 0.2, -0.5, 0, 0, 0, 0.0, 0.0]
r_pos_10 = [-1.35, 0.0, -0.5, 0, 0, 0, 0.0, 0.0]
r_pos_11 = [-1.35, 0.0, -0.5, 0, 0.2, 0, 0.0, 0.0]
r_pos_12 = [-1.35, 0.0, -0.5, -0.3, 0.2, 0, 0.0, 0.0]
r_pos_13 = [-1.35, 0.0, -0.7, -0.3, 0.2, 0, 0.7, 0.7]
r_pos_14 = [-1.35, 0.0, -0.7, 0, 0, 0, 0.7, 0.7]
r_pos_15 = [-1.1, 0.0, 0, -0.3, 0.2, 0, 0.7, 0.7]

l_intial_pos = [0, 0, 0, 0, 0, 0, 0.01, 0.01]
l_pos_1 = [1, 0, 0, 0, 0, 0, 0.01, 0.01]
l_pos_2 = [1.35, 0, -0.4, 0, 0, 0, 0.5, 0.5]
l_pos_3 = [1.4, -0.2, -0.4, 0, 0, 0, 0.5, 0.5]
l_pos_4 = [1.4, -0.2, -0.4, 0, 0, 0, 0.7, 0.7]
l_pos_5 = [1.7, -0.2, -0.4, 0, 0, 0, 0.7, 0.7]
l_pos_6 = [1.7, -0.2, -0.4, 0, 0, 0, 0.01, 0.01]
l_pos_7 = [1.4, -0.2, -0.4, 0, 0, 0, 0.01, 0.01]
l_pos_8 = [1.35, -0.2, -0.4, 0, 0, 0, 0.0, 0.0]
l_pos_9 = [1.35, -0.2, 0.5, 0, 0, 0, 0.0, 0.0]
l_pos_10 = [1.35, 0.0, 0.5, 0, 0, 0, 0.0, 0.0]
l_pos_11 = [1.35, 0.0, 0.5, 0, -0.2, 0, 0.0, 0.0]
l_pos_12 = [1.35, 0.0, 0.5, 0.3, -0.2, 0, 0.0, 0.0]
l_pos_13 = [1.35, 0.0, 0.7, 0.3, -0.2, 0, 0.7, 0.7]

def r_pub_angle(pose2, pose1):
    t1 = np.linspace(pose2[0], pose1[0], 10)
    t2 = np.linspace(pose2[1], pose1[1], 10)
    t3 = np.linspace(pose2[2], pose1[2], 10)
    t4 = np.linspace(pose2[3], pose1[3], 10)
    t5 = np.linspace(pose2[4], pose1[4], 10)
    t6 = np.linspace(pose2[5], pose1[5], 10)
    t7 = np.linspace(pose2[6], pose1[6], 10)
    t8 = np.linspace(pose2[7], pose1[7], 10)
    
    for i in range(9):
        pub_r_shoulder_pan_joint_controller.publish(t1[i])
        pub_r_shoulder_lift_joint_controller.publish(t2[i])
        pub_r_elbow_joint_controller.publish(t3[i])
        pub_r_wrist_1_joint_controller.publish(t4[i])
        pub_r_wrist_2_joint_controller.publish(t5[i])
        pub_r_wrist_3_joint_controller.publish(t6[i])
        pub_r_lfinger_joint_controller.publish(t7[i])
        pub_r_rfinger_joint_controller.publish(t8[i])
        time.sleep(0.5)


def l_pub_angle(pose2, pose1):
    t1 = np.linspace(pose2[0], pose1[0], 10)
    t2 = np.linspace(pose2[1], pose1[1], 10)
    t3 = np.linspace(pose2[2], pose1[2], 10)
    t4 = np.linspace(pose2[3], pose1[3], 10)
    t5 = np.linspace(pose2[4], pose1[4], 10)
    t6 = np.linspace(pose2[5], pose1[5], 10)
    t7 = np.linspace(pose2[6], pose1[6], 10)
    t8 = np.linspace(pose2[7], pose1[7], 10)

    for i in range(9):

        pub_l_shoulder_pan_joint_controller.publish(t1[i])
        pub_l_shoulder_lift_joint_controller.publish(t2[i])
        pub_l_elbow_joint_controller.publish(t3[i])
        pub_l_wrist_1_joint_controller.publish(t4[i])
        pub_l_wrist_2_joint_controller.publish(t5[i])
        pub_l_wrist_3_joint_controller.publish(t6[i])
        pub_l_lfinger_joint_controller.publish(t7[i])
        pub_l_rfinger_joint_controller.publish(t8[i])
        time.sleep(0.5)


def move():
    global pub_r_shoulder_pan_joint_controller, x, pub_r_shoulder_lift_joint_controller, pub_r_elbow_joint_controller, pub_r_wrist_1_joint_controller, pub_r_wrist_2_joint_controller, pub_r_wrist_3_joint_controller, pub_r_lfinger_joint_controller, pub_r_rfinger_joint_controller, pub_l_shoulder_pan_joint_controller, pub_l_shoulder_lift_joint_controller, pub_l_elbow_joint_controller, pub_l_wrist_1_joint_controller, pub_l_wrist_2_joint_controller, pub_l_wrist_3_joint_controller, pub_l_lfinger_joint_controller, pub_l_rfinger_joint_controller

    rospy.init_node('move', anonymous=True)

    pub_r_shoulder_pan_joint_controller = rospy.Publisher(
        '/l_shoulder_pan_joint_controller/command', Float64, queue_size=10)
    pub_r_shoulder_lift_joint_controller = rospy.Publisher(
        '/l_shoulder_lift_joint_controller/command', Float64, queue_size=10)
    pub_r_elbow_joint_controller = rospy.Publisher(
        '/l_elbow_joint_controller/command', Float64, queue_size=10)
    pub_r_wrist_1_joint_controller = rospy.Publisher(
        '/l_wrist_1_joint_controller/command', Float64, queue_size=10)
    pub_r_wrist_2_joint_controller = rospy.Publisher(
        '/l_wrist_2_joint_controller/command', Float64, queue_size=10)
    pub_r_wrist_3_joint_controller = rospy.Publisher(
        '/l_wrist_3_joint_controller/command', Float64, queue_size=10)
    pub_r_lfinger_joint_controller = rospy.Publisher(
        '/l_lfinger_joint_controller/command', Float64, queue_size=10)
    pub_r_rfinger_joint_controller = rospy.Publisher(
        '/l_rfinger_joint_controller/command', Float64, queue_size=10)

    pub_l_shoulder_pan_joint_controller = rospy.Publisher(
        '/r_shoulder_pan_joint_controller/command', Float64, queue_size=10)
    pub_l_shoulder_lift_joint_controller = rospy.Publisher(
        '/r_shoulder_lift_joint_controller/command', Float64, queue_size=10)
    pub_l_elbow_joint_controller = rospy.Publisher(
        '/r_elbow_joint_controller/command', Float64, queue_size=10)
    pub_l_wrist_1_joint_controller = rospy.Publisher(
        '/r_wrist_1_joint_controller/command', Float64, queue_size=10)
    pub_l_wrist_2_joint_controller = rospy.Publisher(
        '/r_wrist_2_joint_controller/command', Float64, queue_size=10)
    pub_l_wrist_3_joint_controller = rospy.Publisher(
        '/r_wrist_3_joint_controller/command', Float64, queue_size=10)
    pub_l_lfinger_joint_controller = rospy.Publisher(
        '/r_lfinger_joint_controller/command', Float64, queue_size=10)
    pub_l_rfinger_joint_controller = rospy.Publisher(
        '/r_rfinger_joint_controller/command', Float64, queue_size=10)

    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    md = ModelState()

    time.sleep(1)
    r_pub_angle(r_intial_pos, r_pos_1)
    time.sleep(2)
    r_pub_angle(r_pos_1, r_pos_2)
    time.sleep(1)
    r_pub_angle(r_pos_2, r_pos_3)
    time.sleep(1)
    r_pub_angle(r_pos_3, r_pos_4)
    time.sleep(1)
    r_pub_angle(r_pos_4, r_pos_5)
    time.sleep(1)
    r_pub_angle(r_pos_5, r_pos_6)
    time.sleep(1)
    r_pub_angle(r_pos_6, r_pos_7)
    time.sleep(1)

    l_pub_angle(l_intial_pos, l_pos_1)
    r_pub_angle(r_pos_7, r_pos_8)
    time.sleep(2)
    l_pub_angle(l_pos_1, l_pos_2)
    r_pub_angle(r_pos_8, r_pos_9)
    time.sleep(1)
    l_pub_angle(l_pos_2, l_pos_3)
    r_pub_angle(r_pos_9, r_pos_10)
    time.sleep(1)
    l_pub_angle(l_pos_3, l_pos_4)
    r_pub_angle(r_pos_10, r_pos_11)
    time.sleep(1)
    l_pub_angle(l_pos_4, l_pos_5)
    r_pub_angle(r_pos_11, r_pos_12)
    time.sleep(1)
    l_pub_angle(l_pos_5, l_pos_6)
    r_pub_angle(r_pos_12, r_pos_13)
    time.sleep(1)
    l_pub_angle(l_pos_6, l_pos_7)
    r_pub_angle(r_pos_13, r_pos_14)
    time.sleep(1)
    r_pub_angle(r_pos_14, r_pos_15)

    l_pub_angle(l_pos_7, l_pos_8)
    time.sleep(1)
    l_pub_angle(l_pos_8, l_pos_9)
    time.sleep(1)
    l_pub_angle(l_pos_9, l_pos_10)
    time.sleep(1)
    l_pub_angle(l_pos_10, l_pos_11)
    time.sleep(1)
    l_pub_angle(l_pos_11, l_pos_12)
    time.sleep(1)
    l_pub_angle(l_pos_12, l_pos_13)

    md.model_name = "pizza_top1"
    md.pose.position.x = x
    md.pose.position.y = 4.25
    md.pose.position.z = 1.46
    pub.publish(md)

    time.sleep(1)
    md.model_name = "pizza"
    for i in range(200):
        x += 0.01
        md.pose.position.x = x
        md.pose.position.y = 4.25
        md.pose.position.z = 1.42
        md.pose.orientation.x = 0
        md.pose.orientation.y = 0
        md.pose.orientation.z = 0
        md.pose.orientation.w = 0
        pub.publish(md)
        time.sleep(0.01)
    md.pose.orientation.x = 0
    md.pose.orientation.y = 0
    md.pose.orientation.z = 0.7
    md.pose.orientation.w = 0.7
    pub.publish(md)


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
