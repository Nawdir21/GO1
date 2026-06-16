#!/usr/bin/env python3
import os, sys, time, rospy
from geometry_msgs.msg import Twist

def clamp(v, lo, hi): return max(lo, min(hi, v))

class Go1Bridge:
    def __init__(self):
        rospy.init_node("cmd_vel_to_go1")

        # --- Params ---
        self.sdk_root  = rospy.get_param("~sdk_root", "/home/ridwan/Downloads/unitree_legged_sdk")
        self.dest_ip   = rospy.get_param("~dest_ip",  "192.168.12.1")
        self.local_port= int(rospy.get_param("~local_port", 8080))
        self.dest_port = int(rospy.get_param("~dest_port",  8082))
        self.max_vx    = float(rospy.get_param("~max_vx",   0.30))
        self.max_vy    = float(rospy.get_param("~max_vy",   0.15))
        self.max_yaw   = float(rospy.get_param("~max_yaw",  0.00))  # 0 = kein Drehen
        self.timeout_s = float(rospy.get_param("~timeout",  0.30))   # Deadman
        self.rate_hz   = int(rospy.get_param("~rate_hz",   200))     # Sende-Frequenz
        self.eps       = 1e-3

        # SDK importieren
        sys.path.append(os.path.join(self.sdk_root, "example_py/lib/python/amd64"))
        sys.path.append(os.path.join(self.sdk_root, "example_py/lib/python/arm64"))
        import robot_interface as sdk
        self.sdk = sdk

        # UDP / Cmd
        self.udp = sdk.UDP(0xEE, self.local_port, self.dest_ip, self.dest_port)
        self.cmd = sdk.HighCmd()
        self.udp.InitCmdData(self.cmd)

        # State
        self.last_cmd = Twist()
        self.last_time = 0.0

        rospy.Subscriber("/cmd_vel", Twist, self._cb_cmd, queue_size=10)
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo("GO1 Bridge ready: dest=%s max(vx=%.2f, vy=%.2f, yaw=%.2f) timeout=%.2fs",
                      self.dest_ip, self.max_vx, self.max_vy, self.max_yaw, self.timeout_s)

    def _cb_cmd(self, m: Twist):
        self.last_cmd = m
        self.last_time = time.time()

    def _send(self, vx, vy, yaw, mode):
        self.cmd.mode = mode            # 2=walk, 0=idle
        self.cmd.gaitType = 1
        self.cmd.velocity = [vx, vy]
        self.cmd.yawSpeed = yaw
        self.cmd.bodyHeight = 0.0
        self.cmd.footRaiseHeight = 0.0
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    def _as_tuple(self, m: Twist):
        vx = clamp(m.linear.x,  -self.max_vx,  self.max_vx)
        vy = clamp(m.linear.y,  -self.max_vy,  self.max_vy)
        wz = clamp(m.angular.z, -self.max_yaw, self.max_yaw)
        return vx, vy, wz

    def run(self):
        rate = rospy.Rate(self.rate_hz)

        # Start: mehrfach Null senden
        for _ in range(50):
            self._send(0.0, 0.0, 0.0, 0)
            time.sleep(0.005)

        while not rospy.is_shutdown():
            now = time.time()
            if now - self.last_time > self.timeout_s:
                # Deadman ausgelöst -> STOP
                vx, vy, wz = 0.0, 0.0, 0.0
                mode = 0
            else:
                vx, vy, wz = self._as_tuple(self.last_cmd)
                moving = (abs(vx) > self.eps) or (abs(vy) > self.eps) or (abs(wz) > self.eps)
                mode = 2 if moving else 0

            self._send(vx, vy, wz, mode)
            rate.sleep()

    def _shutdown(self):
        # Beim Beenden mehrfach Null senden
        for _ in range(80):
            try:
                self._send(0.0, 0.0, 0.0, 0)
                time.sleep(0.005)
            except Exception:
                break

if __name__ == "__main__":
    Go1Bridge().run()
