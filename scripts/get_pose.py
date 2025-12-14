#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

def pose_callback(msg):
    # Extraire position
    x = msg.pose.position.x
    y = msg.pose.position.y

    # Extraire orientation
    q = msg.pose.orientation
    (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    rospy.loginfo(f"Pose actuelle: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad")

def pose_update_listener():
    rospy.init_node('pose_listener', anonymous=True)
    rospy.Subscriber('/robot_pose', PoseStamped, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    pose_update_listener()
