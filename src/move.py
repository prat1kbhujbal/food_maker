import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState
from std_msgs.msg import String
from geometry_msgs.msg import *

x=0.0
def talker():
    global x
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rospy.init_node('conveyor_pub', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    md=ModelState()
    md.model_name="package"
    while not rospy.is_shutdown():
        i=0
        if(i<39):
            for i in range(40):
                print(i)
                x+=0.1
                md.pose.position.x=x/1000 
                md.pose.position.y=0 
                md.pose.position.z=0 
                md.pose.orientation.x = 0
                md.pose.orientation.y = 0
                md.pose.orientation.z = 0
                md.pose.orientation.w = 0
                pub.publish(md) 
        md.pose.position.x=4
        md.pose.position.y=0 
        md.pose.position.z=0 
        md.pose.orientation.x = 0
        md.pose.orientation.y = 0
        md.pose.orientation.z = 0
        md.pose.orientation.w = 0
        pub.publish(md) 
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass