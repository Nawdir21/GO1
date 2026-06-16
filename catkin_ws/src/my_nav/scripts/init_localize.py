#!/usr/bin/env python3
import time, rospy
from std_srvs.srv import Empty

def main():
    rospy.init_node("init_localize", anonymous=True)
    rospy.loginfo("init_localize: waiting for /global_localization ...")
    try:
        rospy.wait_for_service('/global_localization', timeout=60.0)
    except rospy.ROSException:
        rospy.logerr("Service /global_localization not available.")
        return
    # kleine Wartezeit, bis AMCL & Map stabil sind
    time.sleep(2.0)
    try:
        gl = rospy.ServiceProxy('/global_localization', Empty)
        gl()
        rospy.loginfo("init_localize: called /global_localization ✔")
    except Exception as e:
        rospy.logerr("init_localize: service call failed: %s", e)

if __name__ == "__main__":
    main()
