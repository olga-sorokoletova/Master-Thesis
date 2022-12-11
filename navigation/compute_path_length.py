from nav_msgs.msg import Path, Odometry
import math
import rospy
import sys

#TOPIC_path = "/move_base/GlobalPlanner/plan" # for CoHAN
TOPIC_path = "/move_base_node/NavfnROS/plan" # for SMB
TOPIC_ground_truth = '/base_pose_ground_truth'
sub = None
ground_truth_pose, gt_x, gt_y = None, [], []
def path_cb(data):
	global sub
	path_length = 0
	if len(data.poses) > 0:
		for i in range(len(data.poses) - 1):
		    dx = data.poses[i].pose.position.x - data.poses[i+1].pose.position.x
		    dy = data.poses[i].pose.position.y - data.poses[i+1].pose.position.y
		    path_length += math.hypot(dx, dy)
		#sub.unregister()
		print "Global Path Length = " + str(path_length) + " meters"
        	print "Press Ctrl+C to exit."
        	
def groundtruth_cb(data):
    global ground_truth_pose
    if (ground_truth_pose is None):
        ground_truth_pose = [0,0,0]
    gt_x.append(data.pose.pose.position.x)
    gt_y.append(data.pose.pose.position.y)
    path_length = 0
    if len(gt_x) > 0:
		for i in range(len(gt_x) - 1):
		    dx = gt_x[i] - gt_x[i+1]
		    dy = gt_y[i] - gt_y[i+1]
		    path_length += math.hypot(dx, dy)	
		print "Ground Truth Path Length = " + str(path_length) + " meters"
        	print "Press Ctrl+C to exit."
# main
if __name__ == "__main__":

	rospy.init_node('compute_path_length', disable_signals=True)
	rospy.sleep(1)
	
	sub = rospy.Subscriber(TOPIC_path, Path, path_cb)
	gt_sub = rospy.Subscriber(TOPIC_ground_truth, Odometry, groundtruth_cb)
	print "Listening to "+ TOPIC_path
        rospy.spin()

