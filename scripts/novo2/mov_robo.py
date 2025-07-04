#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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
    
    # Defina o ponto da trajetória (posição de destino das juntas)
    point = JointTrajectoryPoint()
    # point.positions = [-1.19, -2.35, -1.33, 0.51, -5.10, -0.02]  # Posições das juntas - GOAL dos textes 2
    # point.positions = [-1.50, -1.82, -1.35, 0.15, -4.93, 0.00]  # Posições das juntas - GOAL dos textes
    # point.positions = [0, -1.57, 0, 0, 0, -1.57]  # Posições das juntas - START
    # point.positions = [0, -1.57, 0, 0, 0, -1.57]  # Posições das juntas - GOAL dos textes
    point.positions = [-0.00993911099235234, -1.591857695963058, -1.183621035071015, -1.5738745434565917, 0.0015515531894383372, -1.5702232128101832]  
    
    point.time_from_start = rospy.Duration(2)  # Tempo em segundos para alcançar a posição
    
    # Adicione o ponto à mensagem de trajetória
    trajectory_msg.points = [point]
    
    # Publique a trajetória no tópico
    rospy.sleep(1)  # Pequeno delay para garantir que o nó está ativo
    pub.publish(trajectory_msg)
    rospy.loginfo("Mensagem de movimento enviada para o UR5")

if __name__ == '__main__':
    try:
        move_ur5()
    except rospy.ROSInterruptException:
        pass



