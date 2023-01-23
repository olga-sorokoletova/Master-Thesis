import sys, os
import rospy
from std_msgs.msg import String

sys.path.append(os.getenv("MARRTINO_APPS_HOME")+"/navigation")

TOPIC_hateb_log = '/move_base/HATebLocalPlannerROS/hateb_log'
hateb_log_sub = None
filename = sys.path[-1]+ "/hateb_log.txt"

def hateb_log_cb(data):
    global hateb_log_sub, filename
    with open(filename, 'a') as f:
        f.write(data.data)
        f.write("\n")
        # print(data)
        # print("end of data")
        # print(data.data)
        # print("end of data.data")
        # print "Number of records = " + str(len(f.readlines()))
    
if __name__ == "__main__":

    rospy.init_node('hateb_log', disable_signals=True)
    rate = rospy.Rate(10)
    rospy.sleep(1)

    if os.path.exists(filename):
        os.remove(filename)

    hateb_log_sub = rospy.Subscriber(TOPIC_hateb_log, String, hateb_log_cb)
    print "Listening to "+ TOPIC_hateb_log
    print "Press Ctrl+C to exit."
    rospy.spin()