#!/usr/bin/env python3

import rospy
import random
from enum import Enum

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class State(Enum):
    FORWARD = 0
    TURNING = 1
    ESCAPE = 2


class AutoExplorer:
    def __init__(self):
        rospy.init_node("auto_explorer", anonymous=True)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.rate = rospy.Rate(10)

        # laser distances
        self.front = float("inf")
        self.left = float("inf")
        self.right = float("inf")

        # behavior
        self.state = State.FORWARD
        self.safe_distance = 0.45

        self.turn_end_time = rospy.Time.now()
        self.wall_start_time = None
        self.wall_timeout = rospy.Duration(3.0)

        self.turn_direction = 1

        rospy.loginfo("Autonomous explorer started")

    def scan_callback(self, scan):
        ranges = scan.ranges

        def clean(v):
            return v if v > 0.05 else float("inf")

        front_vals = ranges[0:15] + ranges[-15:]
        left_vals = ranges[60:100]
        right_vals = ranges[260:300]

        self.front = min(map(clean, front_vals))
        self.left = min(map(clean, left_vals))
        self.right = min(map(clean, right_vals))

    def run(self):
        while not rospy.is_shutdown():
            cmd = Twist()
            now = rospy.Time.now()

            near_wall = (
                self.front < self.safe_distance or
                self.left < self.safe_distance or
                self.right < self.safe_distance
            )

            # =========================
            # FORWARD
            # =========================
            if self.state == State.FORWARD:
                if self.front > self.safe_distance:
                    cmd.linear.x = 0.18
                    cmd.angular.z = 0.0
                    self.wall_start_time = None
                else:
                    self.turn_direction = 1 if self.left > self.right else -1
                    self.turn_end_time = now + rospy.Duration(
                        random.uniform(1.0, 1.6)
                    )
                    self.state = State.TURNING
                    self.wall_start_time = now

            # =========================
            # TURNING
            # =========================
            elif self.state == State.TURNING:
                if now < self.turn_end_time:
                    cmd.linear.x = -0.05
                    cmd.angular.z = 0.7 * self.turn_direction
                else:
                    self.state = State.FORWARD

            # =========================
            # ESCAPE (anti wall-hugging)
            # =========================
            if near_wall and self.wall_start_time:
                if now - self.wall_start_time > self.wall_timeout:
                    cmd.linear.x = -0.12
                    cmd.angular.z = random.choice([-1.6, 1.6])
                    self.wall_start_time = now
                    self.state = State.FORWARD

            self.cmd_pub.publish(cmd)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        AutoExplorer().run()
    except rospy.ROSInterruptException:
        pass

