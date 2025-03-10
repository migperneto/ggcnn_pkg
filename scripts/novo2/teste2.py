#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_ur5():
    rospy.init_node('ur5_move_script', anonymous=True)
    pub = rospy.Publisher('/ur5/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)

    rospy.sleep(1)  # Aguarde o controlador inicializar.

    # Criar a mensagem de trajetória
    trajectory = JointTrajectory()
    trajectory.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", 
                              "elbow_joint", "wrist_1_joint", 
                              "wrist_2_joint", "wrist_3_joint"]

    # Posição inicial
    initial_point = JointTrajectoryPoint()
    initial_point.positions = [0.0, -1.57, 0.0, 0.0, 0.0, -1.57]
    initial_point.time_from_start = rospy.Duration(2.0)

    # Posição intermediária
    intermediate_point = JointTrajectoryPoint()
    intermediate_point.positions = [0.0, -1.56, 1.14, 0.05, 0.0, -2.77]
    intermediate_point.time_from_start = rospy.Duration(5.0)  # 3s após a posição inicial

    # Voltar à posição inicial
    return_point = JointTrajectoryPoint()
    return_point.positions = [0.0, -1.57, 0.0, 0.0, 0.0, -1.57]
    return_point.time_from_start = rospy.Duration(8.0)  # 3s após a posição intermediária

    # Adicionar os pontos à trajetória
    trajectory.points.append(initial_point)
    trajectory.points.append(intermediate_point)
    trajectory.points.append(return_point)

    # Publicar a trajetória
    pub.publish(trajectory)
    rospy.loginfo("Trajetória enviada para o UR5.")

if __name__ == '__main__':
    try:
        move_ur5()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nó finalizado.")