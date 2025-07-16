#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# Listas para armazenar os dados de tempo e posições das juntas
time_data = []
joint_positions = {}
joint_names_ordered = []

# Configurações do plot
fig, ax = plt.subplots()
lines = {} # Dicionário para armazenar as linhas de cada junta no plot

# Função de callback para processar as mensagens do tópico /joint_states
def joint_state_callback(msg):
    current_time = time.time() - start_time # Tempo relativo ao início da execução do script
    time_data.append(current_time)

    # Inicializa joint_positions e joint_names_ordered na primeira mensagem
    if not joint_positions:
        for name in msg.name:
            joint_positions[name] = []
            joint_names_ordered.append(name)
            lines[name], = ax.plot([], [], label=name) # Cria uma linha para cada junta

    for i, name in enumerate(msg.name):
        if name in joint_positions:
            joint_positions[name].append(msg.position[i])
        else:
            # Lidar com o caso de novas juntas aparecerem (improvável para o UR5)
            rospy.logwarn(f"Nova junta '{name}' detectada. Adicionando ao plot.")
            joint_positions[name] = [msg.position[i]]
            joint_names_ordered.append(name)
            lines[name], = ax.plot([], [], label=name)

    # Manter um número razoável de pontos no gráfico para melhor desempenho
    max_points = 500
    if len(time_data) > max_points:
        time_data.pop(0)
        for name in joint_positions:
            if joint_positions[name]: # Garante que a lista não está vazia
                joint_positions[name].pop(0)

# Função para atualizar o gráfico
def update_plot(frame):
    if not time_data:
        return lines.values()

    # Atualiza os dados para cada linha no plot
    for name in joint_names_ordered:
        if name in lines and name in joint_positions:
            lines[name].set_data(time_data, joint_positions[name])

    ax.relim()  # Recalcula os limites dos eixos
    ax.autoscale_view() # Ajusta a visualização para os novos limites
    return lines.values()

def main():
    global start_time
    rospy.init_node('ur5_joint_state_plotter', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)

    start_time = time.time() # Registra o tempo de início

    # Configurações iniciais do plot
    ax.set_title('Variação das Posições das Juntas do UR5')
    ax.set_xlabel('Tempo (s)')
    ax.set_ylabel('Posição (rad)')
    ax.grid(True)
    ax.legend() # Mostra a legenda com os nomes das juntas

    # Cria a animação para atualizar o plot em tempo real
    ani = animation.FuncAnimation(fig, update_plot, interval=50, blit=True, cache_frame_data=False)

    plt.show()

    rospy.spin() # Mantém o nó rodando até ser desligado

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass