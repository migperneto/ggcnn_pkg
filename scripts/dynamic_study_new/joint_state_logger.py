#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState # Importa o tipo de mensagem JointState
import numpy as np # Para manipulação de arrays, se necessário, mas principalmente para impressão

def joint_state_callback(msg):
    """
    Função de callback que é executada toda vez que uma mensagem JointState é recebida.
    """
    rospy.loginfo("--- Joint State Received ---")
    
    # As mensagens JointState contêm três listas: name, position, velocity, e effort.
    # Nem todas as listas estarão sempre preenchidas (ex: effort pode estar vazio).

    rospy.loginfo("Joint Names: %s", msg.name)

    if msg.position: # Verifica se a lista de posições não está vazia
        # Converte para um array NumPy para facilitar a visualização e manipulação, se desejar
        positions = np.array(msg.position) 
        rospy.loginfo("Positions (rad): %s", positions)
        rospy.loginfo("Positions (deg): %s", np.degrees(positions))
    else:
        rospy.loginfo("Positions: (No data)")

    if msg.velocity: # Verifica se a lista de velocidades não está vazia
        velocities = np.array(msg.velocity)
        rospy.loginfo("Velocities (rad/s): %s", velocities)
    else:
        rospy.loginfo("Velocities: (No data)")

    if msg.effort: # Verifica se a lista de esforços não está vazia
        efforts = np.array(msg.effort)
        rospy.loginfo("Efforts (Nm): %s", efforts)
    else:
        rospy.loginfo("Efforts: (No data)")

    rospy.loginfo("----------------------------")

def joint_state_logger():
    """
    Inicializa o nó ROS e subscreve ao tópico /ur5/joint_states.
    """
    # Inicializa o nó ROS. 'anonymous=True' adiciona um número aleatório ao nome
    # para evitar conflitos se várias instâncias do script forem executadas.
    rospy.init_node('joint_state_logger', anonymous=True)

    # Cria um subscriber.
    # Tópico: '/ur5/joint_states' (confirmado como seu tópico ativo)
    # Tipo de mensagem: JointState
    # Callback: joint_state_callback (a função que será chamada com cada mensagem)
    rospy.Subscriber('/ur5/joint_states', JointState, joint_state_callback)

    rospy.loginfo("Joint State Logger iniciado. Escutando o tópico /ur5/joint_states...")

    # Mantém o nó rodando e esperando por mensagens.
    # Isso é essencial para que o subscriber funcione.
    rospy.spin()

if __name__ == '__main__':
    try:
        joint_state_logger()
    except rospy.ROSInterruptException:
        # Captura interrupções do ROS (como Ctrl+C) para fechar o nó graciosamente.
        rospy.loginfo("Joint State Logger encerrado.")