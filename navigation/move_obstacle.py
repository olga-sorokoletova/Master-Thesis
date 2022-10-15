import math, time, sys

import rospy
import actionlib
from threading import Thread

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Pose

import tf

TOPIC_setpose = '/setpose'

def DEG(a):
    return a*180.0/math.pi

def RAD(a):
    return a*math.pi/180.0

def pose_str(p):
    return "%.2f %.2f %.2f DEG" %(p[0], p[1], DEG(p[2]))
    

# main
if __name__ == "__main__":

	rospy.init_node('setstagepose', disable_signals=True)
	rospy.sleep(1)

	if len(sys.argv)<10:
		print("%s <name_of_the_obstacle_1> <name_of_the_obstacle_2> X1_i Y1_i Th_deg1_i X2_i Y2_i Th_deg2_i X_d" %(sys.argv[0]))
		sys.exit(0)

	obstacle1 = sys.argv[1]
	obstacle2 = sys.argv[2]

	# Obstacle 1 initial config
	X1_i = float(sys.argv[3])
	Y1_i = float(sys.argv[4])
	Th_deg1_i = float(sys.argv[5])

	# Obstacle 2 initial config
	X2_i = float(sys.argv[6])
	Y2_i = float(sys.argv[7])
	Th_deg2_i = float(sys.argv[8])

	# Desired x
	X_d = float(sys.argv[9])


	print("Set poses for %s and %s" %(obstacle1, obstacle2))
	TOPIC_setpose_1 = "/" + obstacle1 + TOPIC_setpose
	TOPIC_setpose_2 = "/" + obstacle2 + TOPIC_setpose

	# Current configuration obs1
	X1 = X1_i
	Y1 = Y1_i
	Th_deg1 = Th_deg1_i

	# Current configuration obs2
	X2 = X2_i
	Y2 = Y2_i
	Th_deg2 = Th_deg2_i

	setpose_pub1 = rospy.Publisher(TOPIC_setpose_1, Pose, queue_size=1, latch=True)
	setpose_pub2 = rospy.Publisher(TOPIC_setpose_2, Pose, queue_size=1, latch=True)
	p1 = Pose()
	p2 = Pose()

	while (X1 <= X_d):
		q1 = tf.transformations.quaternion_from_euler(0,0,RAD(Th_deg1))
		q2 = tf.transformations.quaternion_from_euler(0,0,RAD(Th_deg2))

		# obs1
		p1.position.x = X1
		p1.position.y = Y1
		p1.position.z = 0
		p1.orientation.x = q1[0]
		p1.orientation.y = q1[1]
		p1.orientation.z = q1[2]
		p1.orientation.w = q1[3]

		# obs2
		p2.position.x = X2
		p2.position.y = Y2
		p2.position.z = 0
		p2.orientation.x = q2[0]
		p2.orientation.y = q2[1]
		p2.orientation.z = q2[2]
		p2.orientation.w = q2[3]

		print("Setting pose %s: %r" %(TOPIC_setpose_1, p1))
		print("Setting pose %s: %r" %(TOPIC_setpose_2, p2))

		setpose_pub1.publish(p1)
		setpose_pub2.publish(p2)

		# Update
		X1 += 0.2
		X2 += 0.2

		rospy.sleep(1)

	# last step
	p1.position.x = X_d
	setpose_pub1.publish(p1)



