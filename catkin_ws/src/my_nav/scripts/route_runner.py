#!/usr/bin/env python3
import rospy, time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class RouteRunner:
    def __init__(self):
        rospy.init_node("route_runner")
        self.target_m = float(rospy.get_param("~distance_m", 10.0))
        self.vx_nom   = float(rospy.get_param("~vx_nom", 0.25))
        self.pub_vx   = rospy.Publisher("/route/vx_desired", Float32, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self._s_cmd, queue_size=20)
        self.last_t = None
        self.traveled = 0.0
        self.last_vx = 0.0

    def _s_cmd(self, m: Twist):
        self.last_vx = max(0.0, float(m.linear.x))  # nur Vorwärts

    def run(self):
        rospy.loginfo("RouteRunner: target=%.2f m, vx_nom=%.2f", self.target_m, self.vx_nom)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.pub_vx.publish(Float32(self.vx_nom))
            t = rospy.Time.now().to_sec()
            if self.last_t is not None:
                dt = max(0.0, t - self.last_t)
                self.traveled += self.last_vx * dt
            self.last_t = t

            if self.traveled >= self.target_m:
                rospy.loginfo("RouteRunner: reached %.2f m -> stopping", self.traveled)
                self.pub_vx.publish(Float32(0.0))
                time.sleep(1.0)
                return
            rate.sleep()

if __name__ == "__main__":
    RouteRunner().run()
