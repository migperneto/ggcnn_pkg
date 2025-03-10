#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface

'''
Remove os obstáculos do Moveit

'''

def remove_spheres():
    rospy.init_node("remove_spheres", anonymous=True)
    scene = PlanningSceneInterface()
    
    rospy.sleep(1)  # Espera a cena atualizar

    # Remover os objetos pelo nome
    scene.remove_world_object("sphere1")
    scene.remove_world_object("sphere2")

    rospy.sleep(1)  # Aguarda a atualização da cena
    print("Esferas removidas do RViz!")

if __name__ == "__main__":
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        remove_spheres()
    except rospy.ROSInterruptException:
        pass
