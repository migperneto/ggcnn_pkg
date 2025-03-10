#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg      
import geometry_msgs.msg
from math import pi
from tf.transformations import quaternion_from_euler

tau = 2 * pi

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_interface_tutorial", anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")   
    
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",     # /move_group/display_planned_path
         moveit_msgs.msg.DisplayTrajectory,    # moveit_msgs/DisplayTrajectory
        queue_size=10,
    )

    rospy.sleep(2.0)

    print("Reference frame: %s" % group.get_planning_frame())
    print("End effector link: %s" % group.get_end_effector_link())


    # Target position
    target_pose1 = geometry_msgs.msg.Pose()
    orientation = quaternion_from_euler(-1.571, -0.001, -0.012)    # in RPY (radian) --> [-1.571, -0.001, -0.012]
    target_pose1.orientation.x = orientation[0]
    target_pose1.orientation.y = orientation[1]
    target_pose1.orientation.z = orientation[2]
    target_pose1.orientation.w = orientation[3]
    target_pose1.position.x = 0.002     # Translation --> 0.002, 0.191, 1.001
    target_pose1.position.y = 0.191
    target_pose1.position.z = 1.001
    group.set_pose_target(target_pose1)

    # Visualize the planning
    plan = group.plan()
    if plan:
        print("Visualizing plan: SUCCEEDED")
    else:
        print("Visualizing plan: FAILED")

    # Move the group arm
    group.go(wait=True)

    group.stop()
    group.clear_pose_targets()

    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()