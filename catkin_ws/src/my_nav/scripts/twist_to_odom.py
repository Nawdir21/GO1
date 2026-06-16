#!/usr/bin/env python3
import rospy, math, tf
from geometry_msgs.msg import Twist, Quaternion, Pose, PoseWithCovariance, TwistWithCovariance, Point, Vector3
from nav_msgs.msg import Odometry

class TwistToOdom:
    def __init__(self):
        self.base_frame = rospy.get_param("~base_frame_id", "base_link")
        self.odom_frame = rospy.get_param("~odom_frame_id", "odom")
        self.cmd_topic  = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.x=self.y=self.th=0.0; self.vx=self.vy=self.vth=0.0
        self.br = tf.TransformBroadcaster()
        self.pub= rospy.Publisher("/odom", Odometry, queue_size=20)
        rospy.Subscriber(self.cmd_topic, Twist, self.cb, queue_size=50)
        self.last = rospy.Time.now()

    def cb(self, m: Twist):
        self.vx, self.vy, self.vth = m.linear.x, m.linear.y, m.angular.z

    def spin(self):
        r=rospy.Rate(100)
        while not rospy.is_shutdown():
            now=rospy.Time.now(); dt=(now-self.last).to_sec(); self.last=now
            dx  = self.vx*math.cos(self.th)*dt - self.vy*math.sin(self.th)*dt
            dy  = self.vx*math.sin(self.th)*dt + self.vy*math.cos(self.th)*dt
            dth = self.vth*dt
            self.x+=dx; self.y+=dy; self.th+=dth
            q=tf.transformations.quaternion_from_euler(0,0,self.th)
            self.br.sendTransform((self.x,self.y,0.0), q, now, self.base_frame, self.odom_frame)
            od=Odometry(); od.header.stamp=now; od.header.frame_id=self.odom_frame; od.child_frame_id=self.base_frame
            od.pose=PoseWithCovariance(pose=Pose(position=Point(self.x,self.y,0.0), orientation=Quaternion(*q)))
            od.twist=TwistWithCovariance(twist=Twist(linear=Vector3(self.vx,self.vy,0.0), angular=Vector3(0,0,self.vth)))
            self.pub.publish(od); r.sleep()

if __name__=="__main__":
    rospy.init_node("twist_to_odom"); TwistToOdom().spin()
