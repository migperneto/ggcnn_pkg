#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped # Para dados de TF (transformação)
import tf2_ros # Biblioteca mais recente para TF no ROS (preferível ao tf)
# import tf # Se você estiver usando ROS1 Noetic/Melodic e tf legado, pode usar este.
# from geometry_msgs.msg import PoseStamped # Descomente se usar nó de cinemática direta publicando PoseStamped
import time

class DataCollector:
    def __init__(self):
        rospy.init_node('robot_data_plotter', anonymous=True)
        
        self.time_stamps = []
        self.joint_angles_history = [[] for _ in range(6)] # Para 6 juntas do UR5
        self.cartesian_x_history = []
        self.cartesian_y_history = []
        self.cartesian_z_history = []

        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] # Nomes padrão das juntas do UR5

        # Subscribers
        rospy.Subscriber("/ur5/joint_states", JointState, self.joint_states_callback)
        
        # OUVINTE TF2: Para pegar a posição do end-effector (tool0) em relação à base (base_link)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Se usar um nó de cinemática direta publicando PoseStamped:
        # rospy.Subscriber("/ur5/end_effector_pose", PoseStamped, self.pose_callback)

        self.start_time = rospy.Time.now().to_sec()
        rospy.loginfo("Nó robot_data_plotter iniciado. Coletando dados...")

    def joint_states_callback(self, msg):
        current_time = rospy.Time.now().to_sec() - self.start_time
        
        # --- Coleta dos ângulos das juntas ---
        # Garante que time_stamps e joint_angles_history tenham o mesmo número de entradas
        self.time_stamps.append(current_time) 

        for i, name in enumerate(self.joint_names):
            try:
                # Encontra o índice da junta na mensagem recebida
                joint_index = msg.name.index(name)
                self.joint_angles_history[i].append(msg.position[joint_index])
            except ValueError:
                # Isso pode acontecer se a mensagem joint_states contiver menos juntas
                # ou juntas com nomes diferentes dos esperados.
                # Para evitar erros de índice, adicionamos um valor padrão (ex: 0)
                # ou simplesmente pulamos essa junta se não for crítica.
                self.joint_angles_history[i].append(np.nan) # np.nan para indicar dado ausente
                # rospy.logwarn(f"Junta '{name}' não encontrada na mensagem joint_states.")
            
        # --- Coleta da posição cartesiana (usando TF2) ---
        # Tentamos obter a transformação do 'base_link' para 'tool0'
        try:
            # Espera pela transformação mais recente disponível
            # O 'rospy.Duration(0.1)' é um timeout, ajuste se necessário
            transform = self.tf_buffer.lookup_transform('base_link', 'tool0', rospy.Time(0), rospy.Duration(0.1))
            self.cartesian_x_history.append(transform.transform.translation.x)
            self.cartesian_y_history.append(transform.transform.translation.y)
            self.cartesian_z_history.append(transform.transform.translation.z)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # Se a transformação não estiver disponível, ou houver erro, adiciona NaN para manter o tamanho
            # Isso é crucial para que as listas de plotagem tenham o mesmo comprimento que time_stamps,
            # ou você precisará lidar com tamanhos diferentes nos gráficos.
            self.cartesian_x_history.append(np.nan)
            self.cartesian_y_history.append(np.nan)
            self.cartesian_z_history.append(np.nan)
            # rospy.logwarn(f"Falha ao obter transformação: {e}")

    # --- Callbacks para PoseStamped (Alternativa ao TF) ---
    # Se você tiver um nó que publica a pose do end-effector diretamente
    # def pose_callback(self, msg):
    #     current_time = rospy.Time.now().to_sec() - self.start_time
    #     # É importante que esta lista de tempo esteja alinhada com os dados cartesianos
    #     # Se você usar este callback, talvez não precise coletar time_stamps no joint_states_callback
    #     # ou precise de uma estratégia de sincronização.
    #     # Por simplicidade, assumimos que os time_stamps principais vêm do joint_states.
    #     # self.time_stamps.append(current_time) # Cuidado para não duplicar se também for em joint_states_callback
    #     self.cartesian_x_history.append(msg.pose.position.x)
    #     self.cartesian_y_history.append(msg.pose.position.y)
    #     self.cartesian_z_history.append(msg.pose.position.z)

    # CORREÇÃO: Esta função deve estar DENTRO da classe DataCollector!
    def get_data(self):
        # Mantém o nó ROS ativo para receber mensagens
        # Isso bloqueia a execução até que o nó seja encerrado (ex: Ctrl+C)
        rospy.spin() 
        rospy.loginfo("Coleta de dados finalizada. Preparando para plotar...")
        
        # É importante converter as listas de histórico das juntas para arrays NumPy
        # pois cada elemento é uma lista de ângulos de UMA junta ao longo do tempo.
        joint_angles_np = np.array(self.joint_angles_history)
        
        # Certifique-se de que todas as listas de histórico tenham o mesmo comprimento
        # Para evitar erros de plotagem, se houver falhas na coleta de dados TF.
        min_len = min(len(self.time_stamps), len(self.cartesian_x_history), len(joint_angles_np[0]))
        
        return (self.time_stamps[:min_len], 
                joint_angles_np[:, :min_len], # Ajusta todas as linhas (juntas) para o min_len
                self.joint_names, 
                {'x': self.cartesian_x_history[:min_len], 
                 'y': self.cartesian_y_history[:min_len], 
                 'z': self.cartesian_z_history[:min_len]})

# --- Funções de Plotagem (fora da classe, pois recebem os dados já processados) ---
def plot_joint_angles(time_stamps, joint_angles_data, joint_names):
    """
    Gera um gráfico do comportamento dos ângulos das juntas ao longo do tempo.
    """
    plt.figure(figsize=(12, 7)) # Tamanho da figura para melhor visualização
    
    # joint_angles_data agora é um array numpy, então pode-se iterar diretamente
    for i in range(joint_angles_data.shape[0]): # Itera sobre o número de juntas
        plt.plot(time_stamps, joint_angles_data[i, :], label=f'Junta {joint_names[i]}')

    plt.xlabel('Tempo (s)')
    plt.ylabel('Ângulo (rad)')
    plt.title('Comportamento dos Ângulos das Juntas do UR5')
    plt.grid(True)
    plt.legend(loc='best')
    plt.tight_layout() # Ajusta o layout para evitar sobreposição

def plot_cartesian_position(time_stamps, cartesian_positions_data):
    """
    Gera um gráfico do comportamento da posição cartesiana (X, Y, Z) ao longo do tempo.
    """
    plt.figure(figsize=(12, 7))

    plt.plot(time_stamps, cartesian_positions_data['x'], label='Posição X (m)', color='red')
    plt.plot(time_stamps, cartesian_positions_data['y'], label='Posição Y (m)', color='green')
    plt.plot(time_stamps, cartesian_positions_data['z'], label='Posição Z (m)', color='blue')

    plt.xlabel('Tempo (s)')
    plt.ylabel('Posição (m)')
    plt.title('Comportamento da Posição Cartesiana (X, Y, Z) do End-Effector')
    plt.grid(True)
    plt.legend(loc='best')
    plt.tight_layout()

if __name__ == '__main__':
    collector = DataCollector()
    
    # get_data() bloqueará até que o nó ROS seja encerrado
    time_stamps, joint_angles_data, joint_names, cartesian_positions_data = collector.get_data()
    
    # Gerar e exibir os gráficos APÓS a coleta de dados
    if time_stamps: # Verifica se algum dado foi coletado
        plot_joint_angles(time_stamps, joint_angles_data, joint_names)
        plot_cartesian_position(time_stamps, cartesian_positions_data)
        plt.show() # Exibe todos os gráficos que foram criados
        print(f"Gráficos gerados com sucesso com {len(time_stamps)} pontos de dados!")
    else:
        print("Nenhum dado foi coletado. Verifique se o nó ROS está recebendo mensagens.")