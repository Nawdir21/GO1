#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import Twist
from unitree_legged_msgs.msg import HighCmd

HZ = 50.0

# Sicherheitslimits (erst konservativ halten)
MAX_VX = 0.6
MAX_VY = 0.3
MAX_WZ = 1.2
EPS = 0.02

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class CmdVelToHighCmd:
    def __init__(self):
        self.twist = Twist()
        self.pub = rospy.Publisher('/high_cmd', HighCmd, queue_size=50)
        rospy.Subscriber('/cmd_vel', Twist, self.cb)
        self.phase = 0
        self.phase_until = rospy.Time.now() + rospy.Duration(1.5)  # mode 6 für 1.5s
        self.unlocked = False
        self.timer = rospy.Timer(rospy.Duration(1.0/HZ), self.tick)
        rospy.loginfo("cmdvel_to_highcmd: starting unlock sequence (6->1->2)")

    def cb(self, msg):
        self.twist = msg

    def fill_common(self, m):
        # Header/Flags setzen, falls vorhanden
        try:
            m.head = [0xFE, 0xEF]
        except Exception:
            pass
        try:
            # 0x00 = HighLevel laut Doku
            m.levelFlag = 0x00
        except Exception:
            pass
        m.gaitType = 1
        m.speedLevel = 0
        m.footRaiseHeight = 0.06
        m.bodyHeight = 0.0

    def set_vel(self, m, vx, vy, wz):
        vx = clamp(vx, -MAX_VX, MAX_VX)
        vy = clamp(vy, -MAX_VY, MAX_VY)
        wz = clamp(wz, -MAX_WZ, MAX_WZ)
        try:
            m.velocity = [vx, vy]
        except Exception:
            m.velocity[0] = vx
            m.velocity[1] = vy
        m.yawSpeed = wz

    def tick(self, _):
        now = rospy.Time.now()
        m = HighCmd()
        self.fill_common(m)

        # 1) Unlock-Sequenz: mode 6 -> mode 1
        if not self.unlocked:
            if self.phase == 0:
                m.mode = 6  # Stand up (entspricht L + A)
                self.set_vel(m, 0.0, 0.0, 0.0)
                if now >= self.phase_until:
                    self.phase = 1
                    self.phase_until = now + rospy.Duration(1.0)  # mode 1 für 1.0s
            elif self.phase == 1:
                m.mode = 1  # Force stand
                self.set_vel(m, 0.0, 0.0, 0.0)
                if now >= self.phase_until:
                    self.unlocked = True
                    rospy.loginfo("cmdvel_to_highcmd: unlock done, switching to normal control")
            self.pub.publish(m)
            return

        # 2) Normalbetrieb: kein Twist -> stehen; sonst laufen
        vx, vy, wz = self.twist.linear.x, self.twist.linear.y, self.twist.angular.z
        if abs(vx) + abs(vy) + abs(wz) < EPS:
            m.mode = 1  # stehen
            self.set_vel(m, 0.0, 0.0, 0.0)
        else:
            m.mode = 2  # laufen (velocity+yawSpeed)
            self.set_vel(m, vx, vy, wz)

        self.pub.publish(m)

if __name__ == '__main__':
    rospy.init_node('cmdvel_to_highcmd')
    CmdVelToHighCmd()
    rospy.spin()
