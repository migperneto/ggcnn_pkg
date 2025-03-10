#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg

def move_ur5_to_pose():
    # Inicializa o MoveIt! Commander e o nó ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_move_to_pose', anonymous=True)

    # Cria um objeto MoveGroupCommander para o braço robótico UR5
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    #Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

    # Define a pose desejada (posição e orientação)
    val = [-0.423, 0.191, 0.270] 
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = val[0]   # Posição X (metros)
    target_pose.position.y = val[1]   # Posição Y (metros)
    target_pose.position.z = val[2]   # Posição Z (metros)
    quat = [0.0, 0.0, 0.0, 1.0]
    # Orientação definida como um quaternion (w,x,y,z)
    target_pose.orientation.x = quat[0]
    target_pose.orientation.y = quat[1]
    target_pose.orientation.z = quat[2]
    target_pose.orientation.w = quat[3]  # Define a orientação do end-effector

    # Define o alvo no espaço cartesiano
    move_group.set_pose_target(target_pose)

    # Planeja e executa o movimento
    plan = move_group.go(wait=True)  # Planeja e executa diretamente

    # Libera o alvo após o movimento para evitar que ele permaneça bloqueado
    move_group.stop()
    move_group.clear_pose_targets()

    # Finaliza a comunicação com o MoveIt!
    moveit_commander.roscpp_shutdown()
    rospy.loginfo("Movimento concluído!")

if __name__ == '__main__':
    try:
        move_ur5_to_pose()
    except rospy.ROSInterruptException:
        pass
