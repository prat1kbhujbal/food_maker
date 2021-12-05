#!/usr/bin/python2
import rospy
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from numpy import linspace
from math import pi
from sympy import cos, sin, Matrix, zeros
# Set initial joint angles

def get_transformations(q):
	# Initialize the DH parameters derived for the UR10 arm
	a   = [pi/2, 0, 0, pi/2, -pi/2, 0]
	d   = [0.1273,0,0,0.163941,0.1157,0.0922]
	ai  = [0,-0.612,-0.5723,0,0,0]
	arr = [0,0,0,0,0,0]
	# Add a base frame to derive the transformation of each joint w.r.t the base
	T00 = Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
	arr_0 = [T00]
	#ti,i+1, T01,T12,T23... using the matrix template for the DH Parameters
	for i in range (0,len(q)):
		arr[i] = Matrix([[cos(q[i]), -sin(q[i])*cos(a[i]),  sin(q[i])*sin(a[i]),  ai[i]*cos(q[i])],
				 [sin(q[i]),  cos(q[i])*cos(a[i]), -cos(q[i])*sin(a[i]),  ai[i]*sin(q[i])],
				 [0,          sin(a[i]),            cos(a[i]),            d[i]], 
				 [0,          0,                    0,                    1]])
		arr_0.append(arr[i])
	#t0i = t01*t12*t0i
	mat = [arr_0[0]]
	for i in range (0,len(arr_0)-1):			
		mat.append(mat[i]*arr_0[i+1])
	return mat
def cross_prod(z,o):
	mul = zeros(3,1)
	mul[0] =   z[1]*o[2]-z[2]*o[1]
	mul[1] = -(z[0]*o[2]-z[2]*o[0])
	mul[2] =   z[0]*o[1]-z[1]*o[0]
	return mul

def jacobian(trans):
	jacob = []
	n_joint = len(trans)
	O_n = trans[n_joint-1][0:3,3]
	# Calculate jacobian of all joints except joint 3 since it's locked
	for i in range(0, n_joint-1):
		# Omit third joint jacobian calculation
		#if (i == 2):
		#	continue
		# Calculate jacobian using Method 1
		O   = O_n - trans[i][0:3,3]
		z   = trans[i][0:3,2]
		lin_vel = cross_prod(z,O)
		j_each  = Matrix([[lin_vel[0]], [lin_vel[1]], [lin_vel[2]], [z[0]], [z[1]], [z[2]]])
		jacob.append(j_each)
	j = Matrix([[jacob[0], jacob[1], jacob[2], jacob[3], jacob[4],jacob[5]]])	
	return j



def move():
    # Starts a new node
	rospy.init_node('move',anonymous=True)

	pub_r_shoulder_pan_joint_controller = rospy.Publisher('/r_shoulder_pan_joint_controller/command', Float64, queue_size=10)
	pub_r_shoulder_lift_joint_controller = rospy.Publisher('/r_shoulder_lift_joint_controller/command', Float64, queue_size=10)
	pub_r_elbow_joint_controller = rospy.Publisher('/r_elbow_joint_controller/command', Float64, queue_size=10)
	pub_r_wrist_1_joint_controller = rospy.Publisher('/r_wrist_1_joint_controller/command', Float64, queue_size=10)
	pub_r_wrist_2_joint_controller = rospy.Publisher('/r_wrist_2_joint_controller/command', Float64, queue_size=10)
	pub_r_wrist_3_joint_controller = rospy.Publisher('/r_wrist_3_joint_controller/command', Float64, queue_size=10)

	rate = rospy.Rate(10) # 10hz
    
	thetas = [-pi,0,0,0,0,0]
	points = 200
	time = 200
	radius = 100
	delta_time = time / points
# Calculate the transformation matrices for each joint with respect to the base frame (T0,i)
	transforms = get_transformations(thetas)
	# O_n = transforms[len(transforms)-1][0:3,3]
	# theta_des = linspace(-2 * pi, 0, points)
	# x_des = []
	# z_des = []
	# for i in theta_des:
	# 	x_des.append(radius * sin(i) + O_n[0])
	# 	z_des.append(radius * cos(i) + O_n[2])
	#j = jacobian(transforms)

    #Receiveing the user's input
	print("Let's move the robot")
	while not rospy.is_shutdown():
		# for i in range(points):
		# 	pos_now = transforms[len(transforms)-1][0:3,3]
		# 	pos_des = Matrix([[x_des[i]], [O_n[1]], [z_des[i]]])
		# 	delta_p = pos_des - pos_now
		# 	pos_dot = delta_p / delta_time
		# 	# q_dot = jinv * W
		# 	thetas   = j.inv() * Matrix([[pos_dot[0]], [0], [pos_dot[2]], [0], [0], [0]])
		# 	W = j * thetas
		# 	x_position = pos_now[0] + (W[0] * delta_time)
		# 	z_position = pos_now[2] + (W[2] * delta_time)
		# 	thetas = Matrix([thetas[0], thetas[1], thetas[2], thetas[3], thetas[4], thetas[5]])

		# 	transforms = get_transformations(thetas) 	 
		# 	j = jacobian(transforms)

			pub_r_shoulder_pan_joint_controller.publish(thetas[0])
			pub_r_shoulder_lift_joint_controller.publish(thetas[1])
			pub_r_elbow_joint_controller.publish(thetas[2])
			pub_r_wrist_1_joint_controller.publish(thetas[3])
			pub_r_wrist_2_joint_controller.publish(thetas[4])
			pub_r_wrist_3_joint_controller.publish(thetas[5])

			rate.sleep()

if __name__ == '__main__':
	try:
		move()
	except rospy.ROSInterruptException: pass
