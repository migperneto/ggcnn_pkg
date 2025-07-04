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

    # We get the joint values from the group and change some of the values:
    val = [1.3207399614028636, 0.00040854797170375434, -1.3280860057347388, 1.5588447245056152, -1.6864056880149283, -1.680175758495988, -1.5595766828213637]
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = val[0]
    joint_goal[1] = val[1]
    joint_goal[2] = val[2]
    joint_goal[3] = val[3]
    joint_goal[4] = val[4]
    joint_goal[5] = val[5]
    # joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

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
