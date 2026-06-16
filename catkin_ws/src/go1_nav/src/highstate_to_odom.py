#!/usr/bin/env python3
import rospy, math, tf
from nav_msgs.msg import Odometry
from unitree_legged_msgs.msg import HighState

class OdomFromHighState:
    def __init__(self):
        self.last_t = None
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=20)
        self.br  = tf.TransformBroadcaster()
        rospy.Subscriber('/high_state', HighState, self.cb, queue_size=10)
        rospy.loginfo("highstate_to_odom: publishing /odom + TF odom->base_link")

    def cb(self, st):
        t = rospy.Time.now()
        if self.last_t is None:
            self.last_t = t
            return
        dt = (t - self.last_t).to_sec()
        self.last_t = t
        if dt <= 0.0 or dt > 0.1:
            return

        # Aus HighState: yawSpeed, velocity[0]=vx, velocity[1]=vy (Body-Frame)
        try: wz = float(getattr(st, 'yawSpeed', 0.0))
        except: wz = 0.0
        try:
            vx = float(st.velocity[0])
            vy = float(st.velocity[1])
        except:
            vx, vy = 0.0, 0.0

        # Yaw integrieren
        self.yaw += wz * dt
        c = math.cos(self.yaw); s = math.sin(self.yaw)
        self.x += (c*vx - s*vy) * dt
        self.y += (s*vx + c*vy) * dt

        # Odom-Msg
        od = Odometry()
        od.header.stamp = t
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        od.pose.pose.orientation.x, od.pose.pose.orientation.y, od.pose.pose.orientation.z, od.pose.pose.orientation.w = q
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.twist.twist.linear.x  = vx
        od.twist.twist.linear.y  = vy
        od.twist.twist.angular.z = wz
        self.pub.publish(od)

        # TF: odom -> base_link
        self.br.sendTransform(
            (self.x, self.y, 0.0),
            q,
            t,
            'base_link',
            'odom'
        )

if __name__ == '__main__':
    rospy.init_node('highstate_to_odom')
    OdomFromHighState()
    rospy.spin()
