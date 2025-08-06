#!/usr/bin/env python3

import rospy
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

def spawn_objects():
    rospy.init_node('spawn_objects_in_gazebo', anonymous=True)
    
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    rospy.wait_for_service('/gazebo/delete_model') # Wait for delete service
    delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel) # Create delete service proxy

    # Obtenha o caminho base do pacote trajectory_pkg
    try:
        trajectory_pkg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
    except Exception as e:
        rospy.logerr(f"Could not find 'trajectory_pkg' path: {e}")
        return

    objects_to_spawn_delete = [
        {"name": "box_blue", "file": "models/pick_place/bin_blue/model.sdf", "x": -0.8, "y": 0.240476, "z": 1.025},
        {"name": "box_green", "file": "models/pick_place/bin_green/model.sdf", "x": -0.8, "y": -0.009524, "z": 1.025},
        {"name": "box_red", "file": "models/pick_place/bin_red/model.sdf", "x": -0.8, "y": -0.259524, "z": 1.025},
        {"name": "block_blue_1", "file": "models/pick_place/block_blue/model.sdf", "x": -0.427131, "y": 0.003185, "z": 1.045},
        {"name": "block_green_1", "file": "models/pick_place/block_green/model.sdf", "x": -0.560503, "y": -0.221624, "z": 1.045},
        {"name": "block_red_1", "file": "models/pick_place/block_red/model.sdf", "x": -0.571972, "y": 0.200934, "z": 1.045},
        {"name": "block_blue_2", "file": "models/pick_place/block_blue/model.sdf", "x": -0.424412, "y": -0.143902, "z": 1.045},
        {"name": "block_green_2", "file": "models/pick_place/block_green/model.sdf", "x": -0.441868, "y": 0.201144, "z": 1.045},
        {"name": "block_red_2", "file": "models/pick_place/block_red/model.sdf", "x": -0.564329, "y": 0.006137, "z": 1.045},
    ]


    objects_to_spawn = [
        {"name": "box_blue", "file": "models/pick_place/bin_blue/model.sdf", "x": -0.8, "y": 0.240476, "z": 1.025},
        {"name": "box_green", "file": "models/pick_place/bin_green/model.sdf", "x": -0.8, "y": -0.009524, "z": 1.025},
        {"name": "box_red", "file": "models/pick_place/bin_red/model.sdf", "x": -0.8, "y": -0.259524, "z": 1.025},
        {"name": "block_blue_1", "file": "models/pick_place/block_blue/model.sdf", "x": -0.427131, "y": 0.003185, "z": 1.045},
        {"name": "block_green_1", "file": "models/pick_place/block_green/model.sdf", "x": -0.560503, "y": -0.221624, "z": 1.045},
        {"name": "block_red_1", "file": "models/pick_place/block_red/model.sdf", "x": -0.571972, "y": 0.200934, "z": 1.045},
        {"name": "block_blue_2", "file": "models/pick_place/block_blue/model.sdf", "x": -0.424412, "y": -0.143902, "z": 1.045},
        {"name": "block_green_2", "file": "models/pick_place/block_green/model.sdf", "x": -0.441868, "y": 0.201144, "z": 1.045},
        {"name": "block_red_2", "file": "models/pick_place/block_red/model.sdf", "x": -0.564329, "y": 0.006137, "z": 1.045},
    ]


    # First, try to delete existing models
    rospy.loginfo("Attempting to delete existing models...")
    for obj in objects_to_spawn_delete:
        try:
            resp = delete_model_prox(obj["name"])
            if resp.success:
                rospy.loginfo(f"Successfully deleted existing {obj['name']}")
            else:
                rospy.logdebug(f"Could not delete {obj['name']} (might not exist): {resp.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Delete service call failed for {obj['name']}: {e}")

    rospy.sleep(0.5) # Give Gazebo a moment to process deletions

    # Then, spawn the models
    rospy.loginfo("Attempting to spawn models...")
    for obj in objects_to_spawn:
        model_path = os.path.join(trajectory_pkg_path, obj["file"])
        
        if not os.path.exists(model_path):
            rospy.logerr(f"Model file not found: {model_path}")
            continue

        with open(model_path, 'r') as f:
            model_xml = f.read()

        initial_pose = Pose()
        initial_pose.position = Point(obj["x"], obj["y"], obj["z"])
        initial_pose.orientation = Quaternion(0, 0, 0, 1) # Default orientation (no rotation)

        rospy.loginfo(f"Spawning {obj['name']} from {model_path} at x:{obj['x']}, y:{obj['y']}, z:{obj['z']}")
        try:
            resp = spawn_model_prox(obj["name"], model_xml, "", initial_pose, "world")
            if resp.success:
                rospy.loginfo(f"Successfully spawned {obj['name']}")
            else:
                rospy.logwarn(f"Failed to spawn {obj['name']}: {resp.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn service call failed: {e}")

    rospy.loginfo("Finished attempting to spawn all objects.")

if __name__ == '__main__':
    try:
        spawn_objects()
    except rospy.ROSInterruptException:
        pass