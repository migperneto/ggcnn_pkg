#!/usr/bin/env python3

'''
Insere obstáculos no Moveit

'''


import sys
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def add_spheres():
    rospy.init_node("add_spheres_to_scene", anonymous=True)
    scene = PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    
    rospy.sleep(1)  # Aguarda a inicialização da cena
    
    # Definição do primeiro obstáculo esférico
    sphere1_pose = PoseStamped()
    sphere1_pose.header.frame_id = robot.get_planning_frame()
    sphere1_pose.pose.position.x = 0.5  # Posição X
    sphere1_pose.pose.position.y = 0.2  # Posição Y
    sphere1_pose.pose.position.z = 0.3  # Posição Z
    
    scene.add_sphere("sphere1", sphere1_pose, radius=0.1)  # Raio de 10 cm
    
    # Definição do segundo obstáculo esférico
    sphere2_pose = PoseStamped()
    sphere2_pose.header.frame_id = robot.get_planning_frame()
    sphere2_pose.pose.position.x = 0.3
    sphere2_pose.pose.position.y = -0.3
    sphere2_pose.pose.position.z = 0.4
    
    scene.add_sphere("sphere2", sphere2_pose, radius=0.15)  # Raio de 15 cm
    
    rospy.sleep(1)  # Espera a atualização da cena
    print("Esferas adicionadas à cena!")

if __name__ == "__main__":
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        add_spheres()
    except rospy.ROSInterruptException:
        pass
