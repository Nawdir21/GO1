#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def main():
    rospy.init_node("scan_retime")
    in_topic  = rospy.get_param("~in", "/scan")
    out_topic = rospy.get_param("~out", "/scan_sync")
    frame_fix = rospy.get_param("~frame_id", "")  # optional: Frame überschreiben

    pub = rospy.Publisher(out_topic, LaserScan, queue_size=20)

    def cb(msg: LaserScan):
        msg.header.stamp = rospy.Time.now()
        if frame_fix:
            msg.header.frame_id = frame_fix
        pub.publish(msg)

    rospy.Subscriber(in_topic, LaserScan, cb, queue_size=50)
    rospy.loginfo("scan_retime: %s -> %s", in_topic, out_topic)
    rospy.spin()

if __name__ == "__main__":
    main()
