import rosbag
import pickle
import tf

from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    bag = rosbag.Bag("../bag/track2.bag")
    path = {'x': [], 'y': [], 'yaw': []}
    for topic, msg, t in bag.read_messages(topics=['/tracked_pose']):     
        orientation_list = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        _,_,yaw = tf.transformations.euler_from_quaternion(orientation_list)
        
        path['x'].append(msg.pose.position.x)
        path['y'].append(msg.pose.position.y)
        path['yaw'].append(yaw)

    with open("path2.pkl", "wb") as f:
        pickle.dump(path, f)

    print("done")
