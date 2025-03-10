#!/usr/bin/env python3

import sys
import rospy
import moveit_commander

def get_joint_angles():
    # Inicializa o MoveIt! Commander e o nó ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_get_joint_angles', anonymous=True)

    # Criar um objeto MoveGroupCommander para o braço do UR5
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    # Obtém os valores atuais das juntas
    joint_angles = move_group.get_current_joint_values()

    # Exibe os ângulos das juntas no terminal
    rospy.loginfo("Ângulos das Juntas do UR5 (em radianos):")
    for i, angle in enumerate(joint_angles):
        rospy.loginfo(f"Junta {i + 1}: {angle:.4f} rad")

    # Finaliza o MoveIt!
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        get_joint_angles()
    except rospy.ROSInterruptException:
        pass
