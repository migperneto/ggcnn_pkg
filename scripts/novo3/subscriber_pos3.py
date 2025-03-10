#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

# Variável global para armazenar os dados mais recentes das juntas
latest_joint_positions = None
joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

def joint_state_callback(msg):
    global latest_joint_positions
    # Atualiza a variável global com os dados mais recentes
    latest_joint_positions = msg.position
    # latest_joint_positions = [msg.position[i] for i in range(len(joint_names))]


def ur5_position_listener():
    rospy.init_node('ur5_position_listener', anonymous=True)

    # Subscrição ao tópico de estados das juntas
    rospy.Subscriber('/ur5/joint_states', JointState, joint_state_callback)

    rospy.loginfo("Subscriber para o UR5 iniciado. Aguardando dados...")

    # Controla a taxa de logs (uma vez a cada 2 segundos)
    rate = rospy.Rate(2.0)  # 2.0 Hz = 1 mensagem a cada 0.5 segundos

    while not rospy.is_shutdown():
        # Verifica se há dados disponíveis
        if latest_joint_positions is not None:
            rospy.loginfo(f"Posições das Juntas: {list(latest_joint_positions)}")
        else:
            rospy.loginfo("Aguardando dados do tópico /ur5/joint_states...")

        # Aguarda até o próximo ciclo
        rate.sleep()

if __name__ == '__main__':
    try:
        ur5_position_listener()
    except rospy.ROSInterruptException:
        pass
