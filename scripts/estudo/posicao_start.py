#!/usr/bin/env python3

import rospy 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def posicao():
    #Inicializar o nó
    rospy.init_node('def_pos_initial', anonymous=True)

    #Definir o publisher
    pos_pub = rospy.Publisher('/ur5/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)

    #Definir a taxa de publicação
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        #Criar mensagem
        traj = JointTrajectory
        traj.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        #Definir posições
        point = JointTrajectoryPoint
        point.positions = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0] #posições em radianos
        point.time_from_start = rospy.Duration(3.0)

        traj.points.append(point)

        #Publicar o comando
        pos_pub.publish(traj)

        #Escrever no log
        rospy.loginfo('Mensagem Publicada')

if __name__ == '__main__':
    try:
        posicao()
    except rospy.ROSInterruptException:
        pass









