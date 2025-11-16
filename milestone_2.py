#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped


TEST_CASES_DEG = [
    [0, 0, 0, 0, 0],             # Case 1: all zeros 
    [30, -20, 45, 10, 0],        # Case 2: random reach 
]


THEORETICAL_POS = [
    # [x_th, y_th, z_th] for case 1
    [0.0, 0.0, 0.35],
    # [x_th, y_th, z_th] for case 2
    [0.15, 0.20, 0.40],
]


class Milestone2Node:
    def __init__(self):
        rospy.init_node('milestone_2')

        # Publishers for each joint
        self.joint_pubs = [
            rospy.Publisher('/Joint_1/command', Float64, queue_size=10),
            rospy.Publisher('/Joint_2/command', Float64, queue_size=10),
            rospy.Publisher('/Joint_3/command', Float64, queue_size=10),
            rospy.Publisher('/Joint_4/command', Float64, queue_size=10),
            rospy.Publisher('/Joint_5/command', Float64, queue_size=10),
        ]

        # Subscriber to end-effector pose
        self.last_pose = None
        rospy.Subscriber('/end_effector_pose', PoseStamped, self.pose_callback)

        self.rate = rospy.Rate(50)  # 50 Hz

    def pose_callback(self, msg):
        self.last_pose = msg

    def send_joint_angles(self, q_rad):
        """Publish desired joint angles (in radians) to all joints."""
        for i in range(5):
            cmd = Float64()
            cmd.data = float(q_rad[i])
            self.joint_pubs[i].publish(cmd)

    def run(self):
        rospy.loginfo("Milestone 2 node started")

        # Convert test cases to radians
        test_cases = [np.deg2rad(q_deg) for q_deg in TEST_CASES_DEG]

        for idx, q_rad in enumerate(test_cases):
            rospy.loginfo("========== TEST CASE %d ==========", idx+1)
            rospy.loginfo("Desired joint angles (deg): %s", TEST_CASES_DEG[idx])

            # 1) Publish joint angles for some time so the arm reaches steady state
            t_start = rospy.Time.now().to_sec()
            while not rospy.is_shutdown():
                t_now = rospy.Time.now().to_sec()
                if t_now - t_start > 3.0:   # 3 seconds of command
                    break
                self.send_joint_angles(q_rad)
                self.rate.sleep()

            # 2) Wait a little to be sure pose is updated
            rospy.sleep(1.0)

            if self.last_pose is None:
                rospy.logwarn("No end-effector pose received!")
                continue

            x_m = self.last_pose.pose.position.x
            y_m = self.last_pose.pose.position.y
            z_m = self.last_pose.pose.position.z

            rospy.loginfo("Measured EE pose from Gazebo:")
            rospy.loginfo("x = %.4f m, y = %.4f m, z = %.4f m", x_m, y_m, z_m)

            # 3) (Optional) Compare with theoretical FK from MATLAB
            if 0 <= idx < len(THEORETICAL_POS):
                x_th, y_th, z_th = THEORETICAL_POS[idx]
                err_x = x_m - x_th
                err_y = y_m - y_th
                err_z = z_m - z_th
                rospy.loginfo("Theoretical FK EE pose:")
                rospy.loginfo("x_th = %.4f m, y_th = %.4f m, z_th = %.4f m",
                              x_th, y_th, z_th)
                rospy.loginfo("Position error (measured - theoretical):")
                rospy.loginfo("ex = %.4f m, ey = %.4f m, ez = %.4f m",
                              err_x, err_y, err_z)

            rospy.loginfo("======================================")

        rospy.loginfo("All test cases finished. Node will keep spinning...")
        # Keep spinning so you can still see /end_effector_pose, etc.
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = Milestone2Node()
        node.run()
    except rospy.ROSInterruptException:
        pass

