import rosbag
import pickle
import tf
import rospy
from geometry_msgs.msg import PoseStamped

def pose_callback(msg) :
    global path
    path['x'].append(msg.pose.position.x)
    path['y'].append(msg.pose.position.y)
    orientation_list = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
    _,_,yaw = tf.transformations.euler_from_quaternion(orientation_list)
    path['yaw'].append(yaw)

rospy.init_node('pose_to_map')
rospy.Subscriber('tracked_pose', PoseStamped, pose_callback)
path = {'x': [], 'y': [], 'yaw': []}

while not rospy.is_shutdown() :
    

    with open("path.pkl", "wb") as f:
        pickle.dump(path, f)
