#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Point, Quaternion # <--- MODIFIED LINE: Added PoseStamped, Point, Quaternion
from math import pi, tau, dist, floor # dist is not used here but if it was, it would be math.dist
from std_msgs.msg import String
from sensor_msgs.msg import JointState # For gripper control

class UR5PickAndPlace:
    def __init__(self):
        rospy.init_node('ur5_pick_and_place', anonymous=True)

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("manipulator") # Verify your MoveIt! group name

        # Gripper control (Robotiq 85)
        self.gripper_pub = rospy.Publisher('/ur5/gripper_controller/command', JointState, queue_size=1)
        rospy.sleep(0.5) # Give time for publisher to connect

        # Set planning parameters
        self.move_group.set_planning_time(10) # Increased planning time
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_goal_position_tolerance(0.005) # 5 mm
        self.move_group.set_goal_orientation_tolerance(0.01) # ~0.5 degrees
        self.move_group.allow_replanning(True)

        self.end_effector_link = self.move_group.get_end_effector_link()
        rospy.loginfo(f"Using end-effector link: {self.end_effector_link}")

        # Define home/initial joint state (safe, clear view for camera)
        self.joint_home_pose = [0, -pi/2, pi/2, -pi/2, -pi/2, 0] # Adjust as needed

        # Object pose and color subscribers
        rospy.Subscriber('/detected_object_pose', PoseStamped, self.object_pose_callback) # Corrected
        rospy.Subscriber('/detected_object_color', String, self.object_color_callback)

        self.detected_object_pose = None
        self.detected_object_color = None
        self.object_ready_for_pickup = False

        # Define Place Locations for each color (adjust these XYZ coordinates precisely for your Gazebo scene!)
        self.place_orientation = Quaternion(x=0.5, y=0.5, z=-0.5, w=-0.5) # Tool pointing down

        self.place_locations = {
            "red": geometry_msgs.msg.Pose(
                position=Point(x=0.25, y=0.55, z=0.08), # Example coordinates for red box
                orientation=self.place_orientation
            ),
            "green": geometry_msgs.msg.Pose(
                position=Point(x=0.25, y=0.35, z=0.08), # Example coordinates for green box
                orientation=self.place_orientation
            ),
            "blue": geometry_msgs.msg.Pose(
                position=Point(x=0.25, y=0.15, z=0.08), # Example coordinates for blue box
                orientation=self.place_orientation
            )
        }
        # Add the boxes to the planning scene as obstacles
        self._add_boxes_to_scene()

        rospy.loginfo("UR5 Pick and Place Node Initialized.")

    def _add_boxes_to_scene(self):
        rospy.loginfo("Adding colored boxes to planning scene as obstacles...")
        box_size = (0.2, 0.2, 0.1) # Approximate size of your boxes in Gazebo

        # Red Box
        red_box_pose = PoseStamped() # Corrected
        red_box_pose.header.frame_id = "world" # Or "base_link" if boxes are relative to robot
        red_box_pose.pose.position.x = 0.25 # Adjust X, Y, Z to match your Gazebo scene
        red_box_pose.pose.position.y = 0.55
        red_box_pose.pose.position.z = 0.05 # Half of box height + table height
        red_box_pose.pose.orientation.w = 1.0
        self.scene.add_box("red_box_obstacle", red_box_pose, box_size)

        # Green Box
        green_box_pose = PoseStamped() # Corrected
        green_box_pose.header.frame_id = "world"
        green_box_pose.pose.position.x = 0.25
        green_box_pose.pose.position.y = 0.35
        green_box_pose.pose.position.z = 0.05
        green_box_pose.pose.orientation.w = 1.0
        self.scene.add_box("green_box_obstacle", green_box_pose, box_size)

        # Blue Box
        blue_box_pose = PoseStamped() # Corrected
        blue_box_pose.header.frame_id = "world"
        blue_box_pose.pose.position.x = 0.25
        blue_box_pose.pose.position.y = 0.15
        blue_box_pose.pose.position.z = 0.05
        blue_box_pose.pose.orientation.w = 1.0
        self.scene.add_box("blue_box_obstacle", blue_box_pose, box_size)

        rospy.sleep(1.0) # Give time for the scene to update

    def object_pose_callback(self, msg):
        self.detected_object_pose = msg

    def object_color_callback(self, msg):
        self.detected_object_color = msg.data
        self.object_ready_for_pickup = True
        rospy.loginfo("Object color received: {}".format(self.detected_object_color))

    def go_to_joint_state(self, joint_values):
        self.move_group.set_joint_value_target(joint_values)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def go_to_pose_goal(self, pose):
        self.move_group.set_pose_target(pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def open_gripper(self):
        rospy.loginfo("Opening gripper...")
        gripper_msg = JointState()
        gripper_msg.header.stamp = rospy.Time.now()
        gripper_msg.name = ["robotiq_85_left_knuckle_joint"]
        gripper_msg.position = [0.0] # Fully open
        self.gripper_pub.publish(gripper_msg)
        rospy.sleep(1.0)

    def close_gripper(self, value=0.8):
        rospy.loginfo("Closing gripper...")
        gripper_msg = JointState()
        gripper_msg.header.stamp = rospy.Time.now()
        gripper_msg.name = ["robotiq_85_left_knuckle_joint"]
        gripper_msg.position = [value] # Fully closed
        self.gripper_pub.publish(gripper_msg)
        rospy.sleep(1.0)

    def pick(self, target_pose, object_name="target_cube"):
        rospy.loginfo(f"Starting PICK sequence for {object_name}...")

        self.open_gripper()

        gripper_orientation_at_object = Quaternion(x=0.5, y=0.5, z=-0.5, w=-0.5) # Corrected

        pre_grasp_pose = geometry_msgs.msg.Pose()
        pre_grasp_pose.position.x = target_pose.pose.position.x
        pre_grasp_pose.position.y = target_pose.pose.position.y
        pre_grasp_pose.position.z = target_pose.pose.position.z + 0.10
        pre_grasp_pose.orientation = gripper_orientation_at_object

        grasp_pose = geometry_msgs.msg.Pose()
        grasp_pose.position.x = target_pose.pose.position.x
        grasp_pose.position.y = target_pose.pose.position.y
        grasp_pose.position.z = target_pose.pose.position.z - 0.01
        grasp_pose.orientation = gripper_orientation_at_object

        cube_size = (0.04, 0.04, 0.04)
        # Using a PoseStamped for adding object to scene
        object_pose_stamped_for_scene = PoseStamped() # Corrected
        object_pose_stamped_for_scene.header.frame_id = target_pose.header.frame_id
        object_pose_stamped_for_scene.pose = target_pose.pose
        self.scene.add_box(object_name, object_pose_stamped_for_scene, cube_size)
        rospy.sleep(0.1)

        rospy.loginfo("Moving to pre-grasp pose...")
        if not self.go_to_pose_goal(pre_grasp_pose):
            rospy.logerr("Failed to reach pre-grasp pose.")
            self.scene.remove_world_object(object_name)
            return False

        rospy.loginfo("Moving to grasp pose...")
        waypoints = []
        waypoints.append(pre_grasp_pose)
        waypoints.append(grasp_pose)

        (plan, fraction) = self.move_group.compute_cartesian_path(
                               waypoints, 0.01, 0.0
                           )
        if fraction < 1.0:
            rospy.logwarn(f"Failed to plan full Cartesian path for grasp. Fraction: {fraction*100:.2f}%")
            return False

        rospy.loginfo("Executing grasp trajectory...")
        self.move_group.execute(plan, wait=True)
        rospy.sleep(0.5)

        self.close_gripper()

        rospy.loginfo(f"Attaching object '{object_name}' to gripper...")
        self.move_group.attach_object(object_name, self.end_effector_link)
        rospy.sleep(1.0)

        post_grasp_pose = geometry_msgs.msg.Pose()
        post_grasp_pose.position.x = grasp_pose.position.x
        post_grasp_pose.position.y = grasp_pose.position.y
        post_grasp_pose.position.z = grasp_pose.position.z + 0.15
        post_grasp_pose.orientation = grasp_pose.orientation

        waypoints = []
        waypoints.append(grasp_pose)
        waypoints.append(post_grasp_pose)

        (plan, fraction) = self.move_group.compute_cartesian_path(
                               waypoints, 0.01, 0.0
                           )
        if fraction < 1.0:
            rospy.logwarn(f"Failed to plan full Cartesian path for retreat. Fraction: {fraction*100:.2f}%")
            return False

        rospy.loginfo("Executing retreat trajectory...")
        self.move_group.execute(plan, wait=True)
        rospy.sleep(0.5)

        rospy.loginfo(f"PICK sequence for {object_name} completed.")
        return True

    def place(self, place_location_pose, object_name="target_cube"):
        rospy.loginfo(f"Starting PLACE sequence for {object_name}...")

        pre_place_pose = geometry_msgs.msg.Pose()
        pre_place_pose.position.x = place_location_pose.position.x
        pre_place_pose.position.y = place_location_pose.position.y
        pre_place_pose.position.z = place_location_pose.position.z + 0.15
        pre_place_pose.orientation = place_location_pose.orientation

        rospy.loginfo("Moving to pre-place pose...")
        if not self.go_to_pose_goal(pre_place_pose):
            rospy.logerr("Failed to reach pre-place pose.")
            return False

        rospy.loginfo("Moving to place pose...")
        waypoints = []
        waypoints.append(pre_place_pose)
        waypoints.append(place_location_pose)

        (plan, fraction) = self.move_group.compute_cartesian_path(
                               waypoints, 0.01, 0.0
                           )
        if fraction < 1.0:
            rospy.logwarn(f"Failed to plan full Cartesian path for place. Fraction: {fraction*100:.2f}%")
            return False

        rospy.loginfo("Executing place trajectory...")
        self.move_group.execute(plan, wait=True)
        rospy.sleep(0.5)

        self.open_gripper()

        rospy.loginfo(f"Detaching object '{object_name}' from gripper...")
        self.move_group.detach_object(object_name)
        rospy.sleep(1.0)

        post_place_pose = geometry_msgs.msg.Pose()
        post_place_pose.position.x = place_location_pose.position.x
        post_place_pose.position.y = place_location_pose.position.y
        post_place_pose.position.z = place_location_pose.position.z + 0.10
        post_place_pose.orientation = place_location_pose.orientation

        waypoints = []
        waypoints.append(place_location_pose)
        waypoints.append(post_place_pose)

        (plan, fraction) = self.move_group.compute_cartesian_path(
                               waypoints, 0.01, 0.0
                           )
        if fraction < 1.0:
            rospy.logwarn(f"Failed to plan full Cartesian path for post-place retreat. Fraction: {fraction*100:.2f}%")
            return False

        rospy.loginfo("Executing post-place retreat trajectory...")
        self.move_group.execute(plan, wait=True)
        rospy.sleep(0.5)

        rospy.loginfo(f"PLACE sequence for {object_name} completed.")
        return True

    def run(self):
        rospy.loginfo("Moving to home joint state...")
        if not self.go_to_joint_state(self.joint_home_pose):
            rospy.logerr("Failed to reach home joint state. Exiting.")
            return

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            if self.object_ready_for_pickup:
                object_pose_to_pick = self.detected_object_pose
                object_color_to_pick = self.detected_object_color
                self.object_ready_for_pickup = False

                rospy.loginfo(f"Received request to pick up {object_color_to_pick} object at {object_pose_to_pick.pose.position}")

                place_location = self.place_locations.get(object_color_to_pick)
                if place_location is None:
                    rospy.logerr(f"No defined place location for color: {object_color_to_pick}. Skipping.")
                    continue

                object_name_in_scene = f"{object_color_to_pick}_cube"
                if self.pick(object_pose_to_pick, object_name=object_name_in_scene):
                    rospy.loginfo(f"{object_color_to_pick} object picked successfully!")
                    if self.place(place_location, object_name=object_name_in_scene):
                        rospy.loginfo(f"{object_color_to_pick} object placed successfully!")
                        rospy.loginfo("Returning to home position.")
                        self.go_to_joint_state(self.joint_home_pose)
                    else:
                        rospy.logerr(f"Failed to place {object_color_to_pick} object.")
                else:
                    rospy.logerr(f"Failed to pick {object_color_to_pick} object.")
            else:
                rospy.loginfo("Waiting for object detection...")

            rate.sleep()

if __name__ == '__main__':
    try:
        ur5_pp = UR5PickAndPlace()
        ur5_pp.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("MoveIt! Shut down.")