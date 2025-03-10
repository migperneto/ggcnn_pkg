#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from ur_kinematics import forward_kinematics, inverse_kinematics
from std_msgs.msg import Float64MultiArray
from time import sleep

class UR5Controller:
    def __init__(self):
        rospy.init_node('apf_ur5_controller', anonymous=True)

        # UR5 kinematic parameters (DH parameters).
        self.d1 = rospy.get_param('~d1', 0.089159)
        self.a2 = rospy.get_param('~a2', -0.425)
        self.a3 = rospy.get_param('~a3', -0.39225)
        self.d4 = rospy.get_param('~d4', 0.10915)
        self.d5 = rospy.get_param('~d5', 0.09465)
        self.d6 = rospy.get_param('~d6', 0.0823)
        self.dh_params = [self.d1, self.a2, self.a3, self.d4, self.d5, self.d6]

        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        # Subscribers
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        # Publishers
        self.pose_publisher = rospy.Publisher("/ur5_tcp_pose", PoseStamped, queue_size=10)
        self.joint_command_publisher = rospy.Publisher('/scaled_joint_trajectory_controller/command', Float64MultiArray, queue_size=10)
        # Variables
        self.current_joint_angles = None
        self.current_tcp_pose = None

    def joint_states_callback(self, msg):
        """Callback para receber o estado das juntas do robô."""
        self.current_joint_angles = [msg.position[i] for i in range(len(self.joint_names))]
        self.publish_tcp_pose()

    def publish_tcp_pose(self):
      """Publica a pose atual do TCP."""
      if self.current_joint_angles is not None:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "base_link"
        pose = self.forward_kinematics_ros(self.current_joint_angles)
        pose_stamped.pose = pose
        self.pose_publisher.publish(pose_stamped)
        self.current_tcp_pose = pose

    def forward_kinematics_ros(self, joint_angles):
      """Calcula a cinemática direta e retorna a pose do TCP"""
      # Utiliza a biblioteca `ur_kinematics` para calcular a cinemática direta
      T = forward_kinematics(np.array(joint_angles), self.dh_params)

      pose = Pose()
      pose.position.x = T[0, 3]
      pose.position.y = T[1, 3]
      pose.position.z = T[2, 3]

      # Extracting orientation using quaternion
      qx, qy, qz, qw = self.rotation_matrix_to_quaternion(T[:3,:3])
      pose.orientation.x = qx
      pose.orientation.y = qy
      pose.orientation.z = qz
      pose.orientation.w = qw
      return pose

    def rotation_matrix_to_quaternion(self, R):
      """Converte uma matriz de rotação para um quaternion."""
      qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
      qx = (R[2,1] - R[1,2]) / (4 * qw)
      qy = (R[0,2] - R[2,0]) / (4 * qw)
      qz = (R[1,0] - R[0,1]) / (4 * qw)
      return qx, qy, qz, qw

    def move_to_joint_angles(self, joint_angles):
        """Move the robot to specified joint angles."""
        command = Float64MultiArray()
        command.data = joint_angles
        self.joint_command_publisher.publish(command)
        sleep(0.1)

    def get_current_pose(self):
      """Obtém a pose atual do TCP"""
      return self.current_tcp_pose


if __name__ == '__main__':
    try:
        controller = UR5Controller()
        # Sleep for some time so that the joint angles message gets updated.
        sleep(1)
        current_pose = controller.get_current_pose()
        rospy.loginfo("Current TCP Pose: \n{}".format(current_pose))
        # Example of sending commands
        initial_joint_angles = [-0.09, -0.99, 1.66, -0.77, 0.03, 0.06]
        controller.move_to_joint_angles(initial_joint_angles)
        rospy.loginfo("Moving the robot using a joint position command")
        sleep(2)
        current_pose = controller.get_current_pose()
        rospy.loginfo("Current TCP Pose: \n{}".format(current_pose))


        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    