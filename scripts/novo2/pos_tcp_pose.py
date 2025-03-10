#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose

# Callback para processar a mensagem do tópico
def tcp_pose_callback(msg):
    # Extrai a posição cartesiana
    position = msg.position
    orientation = msg.orientation

    # Exibe a posição no log
    rospy.loginfo("Posição TCP no Espaço Cartesiano:")
    rospy.loginfo(f"  x: {position.x:.3f}")
    rospy.loginfo(f"  y: {position.y:.3f}")
    rospy.loginfo(f"  z: {position.z:.3f}")

    # Exibe a orientação (quaternions)
    rospy.loginfo("Orientação TCP (quaternions):")
    rospy.loginfo(f"  x: {orientation.x:.3f}")
    rospy.loginfo(f"  y: {orientation.y:.3f}")
    rospy.loginfo(f"  z: {orientation.z:.3f}")
    rospy.loginfo(f"  w: {orientation.w:.3f}")

def ur5_cartesian_listener():
    # Inicializa o nó ROS
    rospy.init_node('ur5_cartesian_listener', anonymous=True)

    # Subscrição ao tópico que publica a posição cartesiana
    rospy.Subscriber('/tool0', Pose, tcp_pose_callback)     

    rospy.loginfo("Subscriber para o TCP do UR5 iniciado. Aguardando dados...")
    rospy.spin()  # Mantém o nó ativo

if __name__ == '__main__':
    try:
        ur5_cartesian_listener()
    except rospy.ROSInterruptException:
        pass
