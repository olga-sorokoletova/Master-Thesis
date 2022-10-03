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
    
    if len(sys.argv)<6:
    	print("%s <name_of_the_obstacle> X_i Y_i Th_deg_i X_d" %(sys.argv[0]))
    	sys.exit(0)
    	
    obstacle = sys.argv[1]
    X_i = sys.argv[2]
    Y_i = sys.argv[3]
    Th_deg_i = sys.argv[4]
    X_d = sys.argv[5]
    	
    if X_d < X_i:
    	print("<X_d> must be greater or equal than <X_i> (provvisory)")
    	sys.exit(0)
    
    if sys.argv[1] != '_':
    	print("Set pose for %s" %obstacle)
    	TOPIC_setpose = "/" + obstacle + TOPIC_setpose
    
    # Initial configuration
    X = float(X_i)
    Y = float(Y_i)
    Th_deg = float(Th_deg_i)
    
    # Desired x
    X_d = float(X_d)
    
    setpose_pub = rospy.Publisher(TOPIC_setpose, Pose, queue_size=1, latch=True)
    p = Pose()
    	
    while (X <= X_d):
    	q = tf.transformations.quaternion_from_euler(0,0,RAD(Th_deg))
    	
    	p.position.x = X
    	p.position.y = Y
    	p.position.z = 0
    	p.orientation.x = q[0]
    	p.orientation.y = q[1]
    	p.orientation.z = q[2]
    	p.orientation.w = q[3]
    	
    	print("Setting pose %s: %r" %(TOPIC_setpose, p))
    	
    	setpose_pub.publish(p)
    	
    	# Update
    	X += 0.1
    	
    	rospy.sleep(1)
    	
    	
    	
