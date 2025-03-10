#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64MultiArray
from ur_kinematics import forward_kinematics
from time import sleep
import tf
from potential_field import PotentialField, Simulator


class UR5Controller:
 def __init__(self):
     rospy.init_node('apf_ur5_controller', anonymous=True)

     # UR5 kinematic parameters (DH parameters)
     self.d1 = rospy.get_param('~d1', 0.089159)
     self.a2 = rospy.get_param('~a2', -0.425)
     self.a3 = rospy.get_param('~a3', -0.39225)
     self.d4 = rospy.get_param('~d4', 0.10915)
     self.d5 = rospy.get_param('~d5', 0.09465)
     self.d6 = rospy.get_param('~d6', 0.0823)
     self.dh_params = [self.d1, self.a2, self.a3, self.d4, self.d5, self.d6]
     self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

     # APF parameters
     k_att = 1.0
     k_rep = 100.0
     rho_0 = 0.5
     step_size = 0.1
     self.potential_field = PotentialField(k_att, k_rep, rho_0, step_size)
      # Set obstacles
     obstacles = [[0.5, 0.5, 0.8], [0.7, 0.2, 0.6]]
     self.potential_field.set_obstacles(obstacles)

     # Initialize goal
     self.goal = [1.0, 1.0, 1.0]

     # Subscribers
     rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
     # Publishers
     self.pose_publisher = rospy.Publisher("/ur5_tcp_pose", PoseStamped, queue_size=10)
     self.velocity_command_publisher = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
     # Variables
     self.current_joint_angles = None
     self.current_tcp_pose = None
      # Initialize the listener for tf
     self.listener = tf.TransformListener()
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

 def loop(self):
     rate = rospy.Rate(10)
     while not rospy.is_shutdown():
       if self.current_tcp_pose is not None:
         #Get the current tcp position
         current_position = np.array([self.current_tcp_pose.position.x, self.current_tcp_pose.position.y, self.current_tcp_pose.position.z])
         # Calculate force from potential field
         force = self.potential_field.total_force(current_position, self.goal)
         #Transform to a speed vector.
         linear_speed_vector = np.array(force)
          # Transform the vector to the base_link frame.
         (trans, rot) = self.listener.lookupTransform('base_link','tool0', rospy.Time(0))
         rot_mat = tf.transformations.quaternion_matrix(rot)
         linear_speed_vector = np.dot(rot_mat[:3,:3].T, linear_speed_vector)
         # Convert the speed in m/s to joint speeds
         command_speed = self.jacobian_to_joint_speeds(linear_speed_vector, self.current_joint_angles)
         # Create the message and publish to topic.
         msg = Float64MultiArray()
         msg.data = command_speed
         self.velocity_command_publisher.publish(msg)
       rate.sleep()
 def jacobian_to_joint_speeds(self, tcp_speed_vector, joint_angles):
     """Transform a speed vector to joint speeds"""
     # 1. Calcula a matriz Jacobiana.
     jacobian = self.jacobian_from_forward_kinematics(joint_angles)
      # 2. Remove linhas nulas do Jacobiano
     jacobian = jacobian[~np.all(jacobian == 0, axis=1)]
     # 3. Calcula a pseudoinversa do Jacobiano.
     jacobian_pinv = np.linalg.pinv(jacobian)
     # 4. Transforma a velocidade do TCP para velocidades nas juntas.
     joint_speeds = np.dot(jacobian_pinv, tcp_speed_vector)
     return joint_speeds

 def jacobian_from_forward_kinematics(self, joint_angles):
   """Calculates the Jacobian matrix using a central finite difference method"""
   delta_q = 0.00001
   #Initialize matrix
   jacobian = np.zeros((6,6))
   #Get current position
   current_position = self.forward_kinematics_ros(joint_angles)
   for i in range(len(joint_angles)):
     joint_angles_p = list(joint_angles)
     joint_angles_p[i] = joint_angles[i] + delta_q
     joint_angles_n = list(joint_angles)
     joint_angles_n[i] = joint_angles[i] - delta_q
     position_p = self.forward_kinematics_ros(joint_angles_p)
     position_n = self.forward_kinematics_ros(joint_angles_n)
     jacobian[0, i] = (position_p.position.x - position_n.position.x) / (2 * delta_q)
     jacobian[1, i] = (position_p.position.y - position_n.position.y) / (2 * delta_q)
     jacobian[2, i] = (position_p.position.z - position_n.position.z) / (2 * delta_q)
     qxp, qyp, qzp, qwp = self.rotation_matrix_to_quaternion(forward_kinematics(np.array(joint_angles_p), self.dh_params)[:3,:3])
     qxn, qyn, qzn, qwn = self.rotation_matrix_to_quaternion(forward_kinematics(np.array(joint_angles_n), self.dh_params)[:3,:3])
     jacobian[3, i] = (qxp - qxn) / (2 * delta_q)
     jacobian[4, i] = (qyp - qyn) / (2 * delta_q)
     jacobian[5, i] = (qzp - qzn) / (2 * delta_q)
   return jacobian

if __name__ == '__main__':
  try:
    controller = UR5Controller()
    # Sleep for some time so that the joint angles message gets updated.
    sleep(1)
    current_pose = controller.get_current_pose()
    rospy.loginfo("Current TCP Pose: \n{}".format(current_pose))
    rospy.loginfo("Starting APF control loop")
    controller.loop()
  except rospy.ROSInterruptException:
    pass