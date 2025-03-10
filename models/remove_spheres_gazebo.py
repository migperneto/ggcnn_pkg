#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import DeleteModel

def delete_model(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp = delete_model_srv(model_name)
        if resp.success:
            rospy.loginfo(f"Modelo {model_name} removido do Gazebo.")
        else:
            rospy.logerr(f"Falha ao remover {model_name}: {resp.status_message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Erro ao chamar serviço: {e}")

if __name__ == "__main__":
    rospy.init_node("remove_spheres_gazebo")
    delete_model("sphere_obstacle1")
    delete_model("sphere_obstacle2")
