#!/usr/bin/env python3
import rospy, math
from std_msgs.msg import Float32, Bool, Int8
from geometry_msgs.msg import Twist

"""
Sidestep-Controller:
- Keine Drehung (yaw = 0).
- Bei Hindernis vorne: vx = 0, strafe (vy) zur freieren Seite, bis frei.
- Linker Wandabstand wird mit vy gehalten (kein yaw!).
- Vortrieb in Metern kommt weiter aus route_runner (/route/vx_desired).
"""

class AvoidSideStep:
    def __init__(self):
        rospy.init_node("avoidance_controller")

        # --- Tuning-Parameter ---
        self.left_target = rospy.get_param("~left_target", 0.50)  # m, Zielabstand links
        self.k_wall_lat  = rospy.get_param("~k_wall_lat", 0.35)   # Gain für Wand-Halten über vy
        self.vx_nom      = rospy.get_param("~vx_nom", 0.25)       # m/s (wird von route_runner überschrieben)
        self.vx_min      = rospy.get_param("~vx_min", 0.04)       # nicht genutzt, aber da falls später
        self.vy_side     = rospy.get_param("~vy_side", 0.12)      # m/s, Seitwärts-Geschwindigkeit beim Umfahren
        self.vy_max      = rospy.get_param("~vy_max",  0.15)      # Deckel für vy gesamt

        # Schwellen für Front-Kollision
        self.stop_th     = rospy.get_param("~stop_thresh", 0.45)  # m -> in Vermeidungsmodus wechseln
        self.clear_th    = rospy.get_param("~clear_thresh", 0.90) # m -> wieder frei (Hysterese)
        self.side_eps    = rospy.get_param("~side_eps", 0.08)     # m, wie viel „mehr Platz“ nötig ist

        # Sensor-Inputs
        self.frontL = self.frontM = self.frontR = float("inf")
        self.leftd  = self.rightd = float("inf")
        self.blocked= False
        self.free_side = 0        # -1=links frei, +1=rechts frei
        self.vx_des = self.vx_nom

        # interner Zustand
        self.avoiding   = False   # sind wir in Sidestep-Phase?
        self.avoid_dir  = 0       # -1 -> links strafen, +1 -> rechts strafen

        # Subscriptions
        rospy.Subscriber("/obst/front_left",  Float32, lambda m: setattr(self,"frontL",m.data), queue_size=1)
        rospy.Subscriber("/obst/front_mid",   Float32, lambda m: setattr(self,"frontM",m.data), queue_size=1)
        rospy.Subscriber("/obst/front_right", Float32, lambda m: setattr(self,"frontR",m.data), queue_size=1)
        rospy.Subscriber("/obst/left",        Float32, lambda m: setattr(self,"leftd", m.data), queue_size=1)
        rospy.Subscriber("/obst/right",       Float32, lambda m: setattr(self,"rightd",m.data), queue_size=1)
        rospy.Subscriber("/obst/blocked",     Bool,    lambda m: setattr(self,"blocked",bool(m.data)), queue_size=1)
        rospy.Subscriber("/obst/free_side",   Int8,    lambda m: setattr(self,"free_side",int(m.data)), queue_size=1)
        rospy.Subscriber("/route/vx_desired", Float32, lambda m: setattr(self,"vx_des",max(0.0,float(m.data))), queue_size=1)

        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=20)

    def choose_side(self):
        """Wähle Seitenrichtung: +1 = rechts (vy negativ), -1 = links (vy positiv)."""
        # Bevorzugt die Seite mit mehr Platz
        if self.free_side != 0:
            return +1 if self.free_side > 0 else -1
        # Fallback: vergleiche Abstände
        if math.isfinite(self.leftd) or math.isfinite(self.rightd):
            return +1 if (self.rightd > self.leftd) else -1
        # Wenn beides unklar, default nach rechts
        return +1

    def step(self):
        min_front = min(self.frontL, self.frontM, self.frontR)

        vx = self.vx_des   # Standard: vorwärts
        vy = 0.0
        yaw = 0.0          # niemals drehen

        # --- State Machine: Avoiding ja/nein ---
        if self.avoiding:
            # Solange front nicht frei -> nur sidestep
            vx = 0.0
            # vy seitwärts in gewählter Richtung (ROS: +y = links, -y = rechts)
            if self.avoid_dir >= 0:
                vy += -self.vy_side   # rechts
            else:
                vy += +self.vy_side   # links

            # Linken Wandabstand trotzdem halten (überlagert, aber gedeckelt)
            if math.isfinite(self.leftd):
                vy += self.k_wall_lat * (self.left_target - self.leftd)

            # Deckeln
            vy = max(-self.vy_max, min(self.vy_max, vy))

            # Hysterese: erst verlassen, wenn deutlich frei
            if min_front >= self.clear_th:
                self.avoiding = False
                self.avoid_dir = 0

        else:
            # Kein Avoiding: normal fahren + linken Abstand über vy regeln
            if math.isfinite(self.leftd):
                vy += self.k_wall_lat * (self.left_target - self.leftd)
            vy = max(-self.vy_max, min(self.vy_max, vy))

            # Hindernis-Trigger -> in Avoiding wechseln
            if min_front < self.stop_th:
                self.avoiding  = True
                self.avoid_dir = self.choose_side()
                vx = 0.0
                # erstes step wird gleich oben in avoiding verarbeitet

        # Publish
        cmd = Twist()
        cmd.linear.x  = max(0.0, vx)
        cmd.linear.y  = vy
        cmd.angular.z = yaw  # bleibt 0
        self.pub_cmd.publish(cmd)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

if __name__ == "__main__":
    AvoidSideStep().run()
