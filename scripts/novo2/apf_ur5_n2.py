#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray # Importe Float64MultiArray caso queira debuggar
from time import sleep
from potential_field_n2 import PotentialField
from inverse_kinematics import inverse_kinematics_ur5 # Importe seu arquivo aqui

# Parâmetros do robô (pode ser obtido do servidor de parâmetros como no seu código original)
joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

# APF parameters
k_att = None
k_rep = None
rho_0 = None
step_size = None
obstacles = None
goal_position = None
start_position = None

# Variáveis
current_joint_angles = None
path = None

# Publishers
# velocity_command_publisher = None # Agora trajectory_publisher
trajectory_publisher = None

def joint_states_callback(msg):
    """Callback para receber o estado das juntas do robô."""
    global current_joint_angles
    current_joint_angles = [msg.position[i] for i in range(len(joint_names))]

def generate_path():
    """Generates a path using the potential field"""
    global k_att, k_rep, rho_0, step_size, obstacles, goal_position, start_position
    potential_field = PotentialField(k_att, k_rep, rho_0, step_size)
    path = potential_field.simulate(start_position, goal_position, obstacles)
    return path

def move_along_path():
    global path, trajectory_publisher, joint_names
    rate = rospy.Rate(10)  # Defina a frequência de envio dos comandos

    if path is not None:
      trajectory_msg = JointTrajectory()
      trajectory_msg.header.stamp = rospy.Time.now()
      trajectory_msg.joint_names = joint_names
      
      for i, point in enumerate(path):
          try:
            # Calcula os ângulos das juntas com a cinemática inversa
            joint_angles = inverse_kinematics_ur5(point[0], point[1], point[2]) # Use o nome da função de seu arquivo

            if np.isnan(joint_angles).any():
              # print("Skipping this point because there is a nan on joint angles")
              continue

            # Cria um ponto de trajetória
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = joint_angles
            trajectory_point.time_from_start = rospy.Duration(i*0.1) # Tempo de duração para cada ponto

            trajectory_msg.points.append(trajectory_point)
          except ValueError:
            print("Inverse kinematics solution not found")
            continue
      trajectory_publisher.publish(trajectory_msg)
      rospy.loginfo("Path trajectory published.")
    rospy.loginfo(joint_angles)
    

def loop():
    global current_joint_angles, path, start_position
    if current_joint_angles is not None:
        path = generate_path()
        move_along_path()
        rospy.loginfo("Path following complete.")

if __name__ == '__main__':
    try:
        print()
        rospy.init_node('apf_ur5_controller', anonymous=True)
        # APF parameters
        k_att = rospy.get_param('~k_att', 1.0)
        k_rep = rospy.get_param('~k_rep', 0.1)
        rho_0 = rospy.get_param('~rho_0', 0.1)
        step_size = rospy.get_param('~step_size', 0.01)
        obstacles = rospy.get_param('~obstacles', [[-0.2, -0.1915, 1.75], [-0.3, 0.191, 1.65]])
        goal_position = rospy.get_param('~goal_position', [-0.0866, -0.5728, 0.5936])
        #goal_position = rospy.get_param('~goal_position', [0.1353, -0.7152, 0.2828])
        start_position = rospy.get_param('~start_position', [-0.0953, -0.1914, 0.9063])
        # Publishers
        trajectory_publisher = rospy.Publisher("/ur5/eff_joint_traj_controller/command", JointTrajectory, queue_size=10)
        # Subscriber
        rospy.Subscriber("joint_states", JointState, joint_states_callback)

        # Espera até receber uma mensagem sobre os ângulos das juntas
        while current_joint_angles is None:
           rospy.loginfo("Waiting for joint states")
           rospy.sleep(0.1)

        rospy.loginfo("Starting APF path following.")
        loop()

    except rospy.ROSInterruptException:
        pass