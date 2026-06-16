#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf2_ros
import math

def q_from_rpy(r, p, y):
    cy, sy = math.cos(y*0.5), math.sin(y*0.5)
    cr, sr = math.cos(r*0.5), math.sin(r*0.5)
    cp, sp = math.cos(p*0.5), math.sin(p*0.5)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    yq = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return Quaternion(x, yq, z, w)

if __name__ == "__main__":
    rospy.init_node("dummy_odom")
    br = tf2_ros.TransformBroadcaster()
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        t = rospy.Time.now()

        # odom -> base_link = Identität (Roboter steht)
        tf = TransformStamped()
        tf.header.stamp = t
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation = q_from_rpy(0, 0, 0)
        br.sendTransform(tf)

        # optionale /odom-Message (alles 0)
        odom = Odometry()
        odom.header.stamp = t
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation = q_from_rpy(0,0,0)
        odom_pub.publish(odom)

        rate.sleep()
