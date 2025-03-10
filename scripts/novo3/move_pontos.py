#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import pandas as pd


def move_ur5():
    # Inicialize o nó ROS
    rospy.init_node('ur5_trajectory_control', anonymous=True)
    # Configura o publisher para o tópico de controle do UR5
    pub = rospy.Publisher('/ur5/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    
        
    # Crie a mensagem de trajetória
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", 
                                  "elbow_joint", "wrist_1_joint", 
                                  "wrist_2_joint", "wrist_3_joint"]
    
    # Carregar dados do arquivo CSV 
    # positions = pd.read_csv('scripts/joint_angles_created_0.csv')
    # positions = positions.values.tolist()
    
    # Defina as posições para o movimento
    positions = [
        [0.0, -1.57, 1.0, 0.0, 0.0, 0.0],  # Posição inicial
        [0.5, -1.0, 0.5, -0.5, 0.5, 0.0],  # Posição 1
        [1.0, -0.5, 0.0, -1.0, 1.0, 0.5],  # Posição 2
        [0.0, -1.57, 0.0, 1.0, -0.5, 0.5], # Posição 3
        [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]   # Retorno à posição inicial
    ]
   
    
    # Adicione cada posição como um ponto de trajetória com tempo de duração
    for i, pos in enumerate(positions):
        point = JointTrajectoryPoint()
        point.positions = pos
        point.time_from_start = rospy.Duration(3 * (i + 1))  # Define o tempo em que o ponto deve ser alcançado
        trajectory_msg.points.append(point)
    
    # Publique a trajetória no tópico
    rospy.sleep(1)  # Pequeno delay para garantir que o nó está ativo
    pub.publish(trajectory_msg)
    rospy.loginfo("Mensagem de trajetória completa enviada para o UR5")

if __name__ == '__main__':
    try:
        move_ur5()
    except rospy.ROSInterruptException:
        pass

