#!/usr/bin/env python3
import math, rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool, Int8

def sector_min(ranges, a_min, a_inc, deg_lo, deg_hi):
    lo = math.radians(deg_lo); hi = math.radians(deg_hi)
    i0 = max(0, int((lo - a_min) / a_inc))
    i1 = min(len(ranges), int((hi - a_min) / a_inc))
    vals = [r for r in ranges[i0:i1] if math.isfinite(r)]
    return min(vals) if vals else float('inf')

def cb(scan: LaserScan):
    a_min = scan.angle_min
    a_inc = scan.angle_increment
    r = scan.ranges

    # Front in 3 Bins: -30..-10, -10..+10, +10..+30 (Grad)
    fL = sector_min(r, a_min, a_inc, -30, -10)
    fM = sector_min(r, a_min, a_inc, -10, +10)
    fR = sector_min(r, a_min, a_inc, +10, +30)

    # Seiten: Links +60..+120°, Rechts -120..-60° (breit, robust)
    d_left  = sector_min(r, a_min, a_inc, +60, +120)
    d_right = sector_min(r, a_min, a_inc, -120, -60)

    pub_fL.publish(fL); pub_fM.publish(fM); pub_fR.publish(fR)
    pub_L.publish(d_left); pub_R.publish(d_right)

    # Stop / Avoid-Logik (nur Flags & freie Seite aus der Sensorik)
    min_front = min(fL, fM, fR)
    blocked = (min_front < stop_thresh)
    pub_blk.publish(blocked)

    free_side = 0
    if min_front < avoid_thresh:
        if d_left - d_right > side_eps:    free_side = -1  # links freier
        elif d_right - d_left > side_eps:  free_side = +1  # rechts freier
    pub_side.publish(free_side)

def main():
    rospy.init_node("scan_fields")
    global stop_thresh, avoid_thresh, side_eps
    stop_thresh  = rospy.get_param("~stop_thresh", 0.45)  # m: harter Stopp
    avoid_thresh = rospy.get_param("~avoid_thresh", 0.80) # m: Ausweichphase
    side_eps     = rospy.get_param("~side_eps", 0.08)     # m: Toleranz
    topic = rospy.get_param("~scan_topic", "/scan_sync")

    rospy.Subscriber(topic, LaserScan, cb, queue_size=1)

    global pub_fL, pub_fM, pub_fR, pub_L, pub_R, pub_blk, pub_side
    pub_fL  = rospy.Publisher("/obst/front_left",  Float32, queue_size=1)
    pub_fM  = rospy.Publisher("/obst/front_mid",   Float32, queue_size=1)
    pub_fR  = rospy.Publisher("/obst/front_right", Float32, queue_size=1)
    pub_L   = rospy.Publisher("/obst/left",        Float32, queue_size=1)
    pub_R   = rospy.Publisher("/obst/right",       Float32, queue_size=1)
    pub_blk = rospy.Publisher("/obst/blocked",     Bool,    queue_size=1)
    pub_side= rospy.Publisher("/obst/free_side",   Int8,    queue_size=1)  # -1/0/+1

    rospy.spin()

if __name__ == "__main__":
    main()
