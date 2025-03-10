#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_to_initial_position():
    rospy.init_node('ur5_move_to_pose', anonymous=True)
    pub = rospy.Publisher('/ur5/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)

    rospy.sleep(1)  # Aguarde o Gazebo e os plugins inicializarem.

    trajectory = JointTrajectory()
    trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 
                              'elbow_joint', 'wrist_1_joint', 
                              'wrist_2_joint', 'wrist_3_joint']

    point = JointTrajectoryPoint()
    point.positions = [0, -1.57, 0, 0, 0, -1.57]  # Posicione inicial.
    point.time_from_start = rospy.Duration(2.0)  # Tempo para atingir a posição.

    trajectory.points.append(point)
    pub.publish(trajectory)

if __name__ == '__main__':
    try:
        move_to_initial_position()
    except rospy.ROSInterruptException:
        pass
