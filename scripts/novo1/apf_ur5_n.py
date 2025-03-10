#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64MultiArray
from ur_kinematics import forward_kinematics
from time import sleep
import tf
from potential_field_n import PotentialField, Simulator

# UR5 kinematic parameters (DH parameters)
d1 = None
a2 = None
a3 = None
d4 = None
d5 = None
d6 = None
dh_params = None
joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

# APF parameters
k_att = 1.0
k_rep = 100.0
rho_0 = 0.5
step_size = 0.1
obstacles = [[0.5, 0.5, 0.8], [0.7, 0.2, 0.6]]

# Variables
current_joint_angles = None
current_tcp_pose = None
potential_field = None

# Publishers
pose_publisher = None
velocity_command_publisher = None
# Listener
listener = None

def rotation_matrix_to_quaternion(R):
    """Converte uma matriz de rotação para um quaternion."""
    qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
    qx = (R[2,1] - R[1,2]) / (4 * qw)
    qy = (R[0,2] - R[2,0]) / (4 * qw)
    qz = (R[1,0] - R[0,1]) / (4 * qw)
    return qx, qy, qz, qw

def forward_kinematics_ros(joint_angles):
    """Calcula a cinemática direta e retorna a pose do TCP"""
    global dh_params
    # Utiliza a biblioteca `ur_kinematics` para calcular a cinemática direta
    T = forward_kinematics(np.array(joint_angles), dh_params)

    pose = Pose()
    pose.position.x = T[0, 3]
    pose.position.y = T[1, 3]
    pose.position.z = T[2, 3]

     # Extracting orientation using quaternion
    qx, qy, qz, qw = rotation_matrix_to_quaternion(T[:3,:3])
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose

def publish_tcp_pose():
    """Publica a pose atual do TCP."""
    global current_joint_angles, current_tcp_pose, pose_publisher
    if current_joint_angles is not None:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "base_link"
        pose = forward_kinematics_ros(current_joint_angles)
        pose_stamped.pose = pose
        pose_publisher.publish(pose_stamped)
        current_tcp_pose = pose

def joint_states_callback(msg):
    """Callback para receber o estado das juntas do robô."""
    global current_joint_angles
    current_joint_angles = [msg.position[i] for i in range(len(joint_names))]
    publish_tcp_pose()


def jacobian_to_joint_speeds(tcp_speed_vector, joint_angles):
    """Transform a speed vector to joint speeds"""
    jacobian = jacobian_from_forward_kinematics(joint_angles)
    # Remove any rows of zeros
    jacobian = jacobian[~np.all(jacobian == 0, axis=1)]
    #Get the pseudo-inverse of jacobian
    jacobian_pinv = np.linalg.pinv(jacobian)
    # Transform speeds
    joint_speeds = np.dot(jacobian_pinv, tcp_speed_vector)
    return joint_speeds


def jacobian_from_forward_kinematics(joint_angles):
  """Calculates the Jacobian matrix using a central finite difference method"""
  global dh_params
  delta_q = 0.00001
  #Initialize matrix
  jacobian = np.zeros((6,6))
  #Get current position
  current_position = forward_kinematics_ros(joint_angles)
  for i in range(len(joint_angles)):
    joint_angles_p = list(joint_angles)
    joint_angles_p[i] = joint_angles[i] + delta_q
    joint_angles_n = list(joint_angles)
    joint_angles_n[i] = joint_angles[i] - delta_q
    position_p = forward_kinematics_ros(joint_angles_p)
    position_n = forward_kinematics_ros(joint_angles_n)
    jacobian[0, i] = (position_p.position.x - position_n.position.x) / (2 * delta_q)
    jacobian[1, i] = (position_p.position.y - position_n.position.y) / (2 * delta_q)
    jacobian[2, i] = (position_p.position.z - position_n.position.z) / (2 * delta_q)
    qxp, qyp, qzp, qwp = rotation_matrix_to_quaternion(forward_kinematics(np.array(joint_angles_p), dh_params)[:3,:3])
    qxn, qyn, qzn, qwn = rotation_matrix_to_quaternion(forward_kinematics(np.array(joint_angles_n), dh_params)[:3,:3])
    jacobian[3, i] = (qxp - qxn) / (2 * delta_q)
    jacobian[4, i] = (qyp - qyn) / (2 * delta_q)
    jacobian[5, i] = (qzp - qzn) / (2 * delta_q)
  return jacobian

def loop():
    global current_tcp_pose, potential_field, velocity_command_publisher, listener, current_joint_angles
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      if current_tcp_pose is not None and current_joint_angles is not None:
        #Get the current tcp position
        current_position = np.array([current_tcp_pose.position.x, current_tcp_pose.position.y, current_tcp_pose.position.z])
        # Calculate force from potential field
        force = potential_field.total_force(current_position, [1.0, 1.0, 1.0])
        #Transform to a speed vector.
        linear_speed_vector = np.array(force)
        # Transform the vector to the base_link frame.
        (trans, rot) = listener.lookupTransform('base_link','tool0', rospy.Time(0))
        rot_mat = tf.transformations.quaternion_matrix(rot)
        linear_speed_vector = np.dot(rot_mat[:3,:3].T, linear_speed_vector)
         # Convert the speed in m/s to joint speeds
        command_speed = jacobian_to_joint_speeds(linear_speed_vector, current_joint_angles)
        # Create the message and publish to topic.
        msg = Float64MultiArray()
        msg.data = command_speed
        velocity_command_publisher.publish(msg)
      rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('apf_ur5_controller', anonymous=True)
        # UR5 kinematic parameters (DH parameters)
        d1 = rospy.get_param('~d1', 0.089159)
        a2 = rospy.get_param('~a2', -0.425)
        a3 = rospy.get_param('~a3', -0.39225)
        d4 = rospy.get_param('~d4', 0.10915)
        d5 = rospy.get_param('~d5', 0.09465)
        d6 = rospy.get_param('~d6', 0.0823)
        dh_params = [d1, a2, a3, d4, d5, d6]
        # APF parameters
        k_att = 1.0
        k_rep = 100.0
        rho_0 = 0.5
        step_size = 0.1
        # Potential field instance
        potential_field = PotentialField(k_att, k_rep, rho_0, step_size)
        # Set obstacles
        potential_field.set_obstacles(obstacles)
        # Publishers
        pose_publisher = rospy.Publisher("/ur5_tcp_pose", PoseStamped, queue_size=10)
        velocity_command_publisher = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        # Subscriber
        rospy.Subscriber("/joint_states", JointState, joint_states_callback)
        # Listener
        listener = tf.TransformListener()
        # Sleep for some time so that the joint angles message gets updated.
        sleep(1)
        rospy.loginfo("Starting APF control loop")
        loop()
    except rospy.ROSInterruptException:
        pass