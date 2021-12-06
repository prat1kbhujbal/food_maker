import rospy 
from gazebo_msgs.msg import ModelState
from std_msgs.msg import String
from geometry_msgs.msg import *
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState


def state0_callback(state_msg):

  print("done")
  rospy.loginfo("process value is %f", state_msg.process_value)

def callback(subdata):
    rospy.loginfo("command is %f", subdata.data)
    print(subdata.data)

def listener():

    print("Listening")
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/l_elbow_joint_controller/command', Float64, callback)
    rospy.Subscriber('/l_elbow_joincontroller/state', JointControllerState, state0_callback)

    print("made node")
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	    rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
	
  	print("list")
    except rospy.ROSInterruptException:
  	print("inerrupt")
        pass
