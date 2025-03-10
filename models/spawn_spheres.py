#!/usr/bin/env python
import rospy
import os
import sys
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn_sphere(model_name, x, y, z):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        # Lê o arquivo SDF
        # model_path = os.path.expanduser(f'~/models/{model_name}.sdf')  # Ajuste o caminho
        model_path = os.path.expanduser(f'~/miguel_ws/src/trajectory_pkg/models/{model_name}.sdf')
        with open(model_path, 'r') as f:
            model_xml = f.read()

        # Define a posição
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        # Chama o serviço de spawn
        resp = spawn_model(model_name, model_xml, "/", pose, "world")
        if resp.success:
            rospy.loginfo(f"{model_name} spawnado com sucesso!")
        else:
            rospy.logerr(f"Falha ao spawnar {model_name}: {resp.status_message}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Erro ao chamar serviço: {e}")

if __name__ == "__main__":
    rospy.init_node("spawn_spheres")
    x1,y1,z1 = [-0.4, 0.0, 0.75] # Obstáculo 1
    x2,y2,z2 = [-0.4, 0.191, 0.65]  # Obstáculo 2
    spawn_sphere("sphere_obstacle1", x1, y1, z1+1.015)
    spawn_sphere("sphere_obstacle2", x2, y2, z2+1.015)


