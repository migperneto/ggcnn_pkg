#!/usr/bin/env python3

'''
script completo em Python para o ROS que envie torques constantes para o seu UR5 no Gazebo e, ao mesmo 
tempo, colete os dados de posição, velocidade e os próprios torques aplicados para gerar gráficos detalhados.
'''

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
import time # Para controlar o tempo de coleta de dados

# --- Variáveis Globais para Armazenamento de Dados ---
# Listas para armazenar os dados de tempo, posição, velocidade e torque
time_data = []
joint_positions_data = [] # q
joint_velocities_data = [] # dq
applied_torques_data = [] # tau (o torque que estamos enviando)

# Torques constantes a serem aplicados (6 juntas)
# Exemplo: aplicando 10 Nm na junta 1 e 5 Nm na junta 2
# Ajuste esses valores conforme a necessidade dos seus testes
CONSTANT_TORQUES = np.array([-100, 0, 0, 0, 0, 0])

# Duração da simulação em segundos
SIMULATION_DURATION = 5.0 # simular por 10 segundos

# --- Callback para o tópico /joint_states ---
def joint_states_callback(msg):
    # O tópico /joint_states do Gazebo pode ter um monte de juntas,
    # incluindo as não-UR5 (como fixas, etc.).
    # O /ur5/joint_states é mais específico, mas vamos usar /joint_states
    # e filtrar pelas juntas do UR5.

    # Nomes das juntas do UR5 na ordem esperada (verifique no seu URDF/Gazebo)
    # A ordem aqui DEVE corresponder à ordem dos seus CONSTANT_TORQUES
    ur5_joint_names = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]

    current_q = np.zeros(6)
    current_dq = np.zeros(6)

    # Encontrar os dados das juntas do UR5 na mensagem JointState
    for i, name in enumerate(ur5_joint_names):
        try:
            idx = msg.name.index(name)
            current_q[i] = msg.position[idx]
            current_dq[i] = msg.velocity[idx]
        except ValueError:
            rospy.logwarn_throttle(5, f"Junta {name} não encontrada na mensagem /joint_states.")
            return # Se uma junta principal não for encontrada, algo está errado, saia.

    # Armazenar os dados se a simulação estiver ativa
    if rospy.get_time() < start_time + SIMULATION_DURATION:
        time_data.append(rospy.get_time() - start_time)
        joint_positions_data.append(current_q)
        joint_velocities_data.append(current_dq)
        applied_torques_data.append(CONSTANT_TORQUES.copy()) # Armazenar uma cópia do torque aplicado

# --- Função Principal ---
def main():
    global start_time # Acessar a variável global para o tempo de início da simulação

    rospy.init_node('ur5_dynamics_tester', anonymous=True)

    # Publicador de torque para o UR5
    # O tópico /ur5/simple_effort_controller/command aceita Float64MultiArray
    # Certifique-se de que o Gazebo está rodando e o controlador de esforço está carregado
    torque_pub = rospy.Publisher('/ur5/simple_effort_controller/command', Float64MultiArray, queue_size=10)

    # Assinante para o tópico de estado das juntas
    rospy.Subscriber('/ur5/joint_states', JointState, joint_states_callback)

    # Dar um tempo para o ROS inicializar as conexões
    rospy.sleep(1.0)
    rospy.loginfo("Nó ur5_dynamics_tester iniciado. Começando a simulação...")

    # Mensagem de torque a ser publicada
    torque_msg = Float64MultiArray()
    torque_msg.data = CONSTANT_TORQUES.tolist() # Converter para lista para a mensagem ROS

    rate = rospy.Rate(100) # Publicar torques a 100 Hz (frequência de controle)

    start_time = rospy.get_time() # Registrar o tempo de início da simulação
    
    # Loop principal de publicação de torque
    while not rospy.is_shutdown() and (rospy.get_time() - start_time) < SIMULATION_DURATION:
        torque_pub.publish(torque_msg)
        rate.sleep()

    rospy.loginfo("Simulação encerrada. Gerando gráficos...")

    # --- Processamento e Plotagem dos Dados ---
    if not time_data:
        rospy.logerr("Nenhum dado coletado. Certifique-se de que o Gazebo está rodando e publicando /joint_states.")
        return

    # Converter listas para arrays NumPy para fácil manipulação
    time_data_np = np.array(time_data)
    joint_positions_data_np = np.array(joint_positions_data)
    joint_velocities_data_np = np.array(joint_velocities_data)
    applied_torques_data_np = np.array(applied_torques_data)

    num_joints = joint_positions_data_np.shape[1] # Deve ser 6

    # Nomes das juntas para os gráficos
    plot_joint_names = [f'Junta {i+1}' for i in range(num_joints)]

    # --- Processamento e Plotagem dos Dados ---
    if not time_data:
        rospy.logerr("Nenhum dado coletado. Certifique-se de que o Gazebo está rodando e publicando /joint_states.")
        return

    # Converter listas para arrays NumPy para fácil manipulação
    time_data_np = np.array(time_data)
    joint_positions_data_np = np.array(joint_positions_data)
    joint_velocities_data_np = np.array(joint_velocities_data)
    applied_torques_data_np = np.array(applied_torques_data)

    num_joints = joint_positions_data_np.shape[1] # Deve ser 6

    # Nomes descritivos para as juntas
    ur5_display_joint_names = [
        'Shoulder Pan', 'Shoulder Lift', 'Elbow',
        'Wrist 1', 'Wrist 2', 'Wrist 3'
    ]

    # Crie uma figura grande para todos os gráficos
    # Teremos num_joints linhas e 2 colunas (Posição/Velocidade, Torque)
    fig, axes = plt.subplots(num_joints, 2, figsize=(18, 4 * num_joints)) # Ajuste o figsize

    # Adiciona um título geral para a figura
    fig.suptitle('UR5 Dynamic Response in Gazebo with Constant Torques', y=1.02, fontsize=16)

    for i in range(num_joints):
        # --- Plotagem de Posição e Velocidade na Primeira Coluna ---
        ax_pos_vel = axes[i, 0] # Acesse o subplot na linha 'i', coluna 0

        ax_pos_vel.plot(time_data_np, joint_positions_data_np[:, i], label=f'Posição (rad)', color='blue')
        ax_pos_vel.plot(time_data_np, joint_velocities_data_np[:, i], label=f'Velocidade (rad/s)', color='orange', linestyle='--')
        
        ax_pos_vel.set_title(f'{ur5_display_joint_names[i]} - Posição e Velocidade')
        ax_pos_vel.set_ylabel('Valores (rad / rad/s)')
        ax_pos_vel.grid(True)
        if i == 0: # Legenda apenas na primeira linha
            ax_pos_vel.legend()
        if i == num_joints - 1: # Rótulo X apenas na última linha
            ax_pos_vel.set_xlabel('Tempo (s)')

        # --- Plotagem de Torque Aplicado na Segunda Coluna ---
        ax_torque = axes[i, 1] # Acesse o subplot na linha 'i', coluna 1

        ax_torque.plot(time_data_np, applied_torques_data_np[:, i], label=f'Torque Aplicado (Nm)', color='green', linestyle=':')
        
        ax_torque.set_title(f'{ur5_display_joint_names[i]} - Torque')
        ax_torque.set_ylabel('Torque (Nm)')
        ax_torque.grid(True)
        if i == 0: # Legenda apenas na primeira linha
            ax_torque.legend()
        if i == num_joints - 1: # Rótulo X apenas na última linha
            ax_torque.set_xlabel('Tempo (s)')

    plt.tight_layout(rect=[0, 0.03, 1, 0.98]) # Ajusta o layout
    plt.show()

    rospy.loginfo("Gráficos gerados. Encerrando nó.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass