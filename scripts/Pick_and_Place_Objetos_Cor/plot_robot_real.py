#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped # Para dados de TF (transformação)
import tf2_ros # Biblioteca mais recente para TF no ROS (preferível ao tf)
# import tf # Se você estiver usando ROS1 Noetic/Melodic e tf legado, pode usar este.
# from geometry_msgs.msg import PoseStamped # Descomente se usar nó de cinemática direta publicando PoseStamped

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
        
        # IMPORTANTE: Garanta que esta linha esteja presente e não comentada!
        self.start_time = rospy.Time.now().to_sec() 
        rospy.loginfo("Nó robot_data_plotter iniciado. Coletando dados...")

    def joint_states_callback(self, msg):
        current_time = rospy.Time.now().to_sec() - self.start_time
        
        # --- Coleta dos ângulos das juntas ---
        # Adiciona o timestamp. Importante que seja aqui para alinhar com os dados de junta.
        self.time_stamps.append(current_time) 

        for i, name in enumerate(self.joint_names):
            try:
                # Encontra o índice da junta na mensagem recebida
                joint_index = msg.name.index(name)
                self.joint_angles_history[i].append(msg.position[joint_index])
            except ValueError:
                # Se a junta não for encontrada ou a mensagem estiver incompleta
                self.joint_angles_history[i].append(np.nan) 
            
        # --- Coleta da posição cartesiana (usando TF2) ---
        # Tenta obter a transformação do 'base_link' para 'tool0'
        # Isso precisa ser feito para CADA timestamp, ou as listas cartesianas
        # ficarão desalinhadas com os time_stamps e joint_angles_history.
        try:
            # Espera pela transformação mais recente disponível no timestamp da mensagem joint_states
            # rospy.Time(0) pega a transformação mais recente disponível.
            # rospy.Time.now() pode ser ligeiramente diferente do tempo da mensagem.
            # O timeout de 0.05 segundos evita que o callback bloqueie por muito tempo.
            transform = self.tf_buffer.lookup_transform('base_link', 'tool0', rospy.Time(0), rospy.Duration(0.05))
            self.cartesian_x_history.append(transform.transform.translation.x)
            self.cartesian_y_history.append(transform.transform.translation.y)
            self.cartesian_z_history.append(transform.transform.translation.z)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # Se a transformação não estiver disponível, ou houver erro, adiciona NaN
            # para manter o mesmo comprimento que time_stamps.
            self.cartesian_x_history.append(np.nan)
            self.cartesian_y_history.append(np.nan)
            self.cartesian_z_history.append(np.nan)
            # rospy.logdebug(f"Falha ao obter transformação em {current_time:.2f}s: {e}") # Usar debug para não encher o console

    # --- Callbacks para PoseStamped (Alternativa ao TF) ---
    # Se você tiver um nó que publica a pose do end-effector diretamente
    # def pose_callback(self, msg):
    #     current_time = rospy.Time.now().to_sec() - self.start_time
    #     # Se usar este callback, precisará ajustar a lógica de time_stamps para sincronizar.
    #     # A maneira mais simples é ter um único callback mestre que dispara tudo,
    #     # ou usar mensagens sincronizadas do ROS (MessageFilter).
    #     self.cartesian_x_history.append(msg.pose.position.x)
    #     self.cartesian_y_history.append(msg.pose.position.y)
    #     self.cartesian_z_history.append(msg.pose.position.z)

    def get_current_data(self):
        """
        Retorna uma cópia dos dados coletados até o momento.
        Usado para plotagem em tempo real.
        """
        # Garante que todas as listas tenham o mesmo comprimento para plotagem
        # O np.array(self.joint_angles_history) pode ter sub-listas de comprimentos diferentes
        # se algumas juntas falharam na coleta. Por isso, precisamos de cuidado.
        
        # Encontra o menor comprimento entre todas as listas de histórico
        lengths = [len(self.time_stamps), len(self.cartesian_x_history)]
        for jl in self.joint_angles_history:
            lengths.append(len(jl))
        min_len = min(lengths)
        
        # Retorna as fatias das listas até o menor comprimento
        joint_angles_sliced = [angles[:min_len] for angles in self.joint_angles_history]
        
        return (self.time_stamps[:min_len], 
                joint_angles_sliced, 
                self.joint_names, 
                {'x': self.cartesian_x_history[:min_len], 
                 'y': self.cartesian_y_history[:min_len], 
                 'z': self.cartesian_z_history[:min_len]})

# --- Funções de Plotagem ---
class RealtimePlotter:
    def __init__(self, collector):
        self.collector = collector
        self.fig_joints, self.ax_joints = plt.subplots(figsize=(12, 7))
        self.fig_cartesian, self.ax_cartesian = plt.subplots(figsize=(12, 7))

        self.line_joints = []
        for i, name in enumerate(self.collector.joint_names):
            line, = self.ax_joints.plot([], [], label=f'Junta {name}')
            self.line_joints.append(line)
        self.ax_joints.set_xlabel('Tempo (s)')
        self.ax_joints.set_ylabel('Ângulo (rad)')
        self.ax_joints.set_title('Comportamento dos Ângulos das Juntas do UR5')
        self.ax_joints.grid(True)
        self.ax_joints.legend(loc='best')
        self.ax_joints.set_xlim(0, 10) # Limite inicial X, será ajustado
        self.ax_joints.set_ylim(-3.2, 3.2) # Limite inicial Y (aprox -pi a pi)

        self.line_x, = self.ax_cartesian.plot([], [], label='Posição X (m)', color='red')
        self.line_y, = self.ax_cartesian.plot([], [], label='Posição Y (m)', color='green')
        self.line_z, = self.ax_cartesian.plot([], [], label='Posição Z (m)', color='blue')
        self.ax_cartesian.set_xlabel('Tempo (s)')
        self.ax_cartesian.set_ylabel('Posição (m)')
        self.ax_cartesian.set_title('Comportamento da Posição Cartesiana (X, Y, Z) do End-Effector')
        self.ax_cartesian.grid(True)
        self.ax_cartesian.legend(loc='best')
        self.ax_cartesian.set_xlim(0, 12) # Limite inicial X, será ajustado
        self.ax_cartesian.set_ylim(-1, 1) # Limite inicial Y, será ajustado

        plt.tight_layout() # Ajusta o layout para evitar sobreposição

    def update_plots(self):
        """
        Atualiza os dados dos gráficos com as informações mais recentes.
        """
        time_stamps, joint_angles_data, _, cartesian_positions_data = self.collector.get_current_data()

        if not time_stamps: # Se não houver dados, não atualiza
            return

        # Atualiza o gráfico de ângulos das juntas
        for i, line in enumerate(self.line_joints):
            # Garante que a linha só tente plotar até o min_len real daquela junta
            line.set_data(time_stamps, joint_angles_data[i])
        
        # Ajusta os limites X do gráfico de juntas (rolagem)
        self.ax_joints.set_xlim(max(0, time_stamps[-1] - 10), time_stamps[-1] + 1) # Mostra os últimos 10s + 1s de margem
        self.ax_joints.relim() # Recalcula os limites de dados
        self.ax_joints.autoscale_view() # Ajusta a visualização para os novos limites

        # Atualiza o gráfico de posição cartesiana
        self.line_x.set_data(time_stamps, cartesian_positions_data['x'])
        self.line_y.set_data(time_stamps, cartesian_positions_data['y'])
        self.line_z.set_data(time_stamps, cartesian_positions_data['z'])

        # Ajusta os limites X do gráfico cartesiano (rolagem)
        self.ax_cartesian.set_xlim(max(0, time_stamps[-1] - 10), time_stamps[-1] + 1) # Mostra os últimos 10s + 1s de margem
        self.ax_cartesian.relim()
        self.ax_cartesian.autoscale_view()

        # Desenha as figuras e pausa brevemente para permitir interação com a GUI e ROS
        self.fig_joints.canvas.draw()
        self.fig_cartesian.canvas.draw()
        plt.pause(0.001) # Pausa mínima para permitir que a GUI seja atualizada

if __name__ == '__main__':
    plt.ion() # Habilita o modo interativo do Matplotlib

    collector = DataCollector()
    plotter = RealtimePlotter(collector)

    rate = rospy.Rate(10) # Taxa de atualização dos gráficos (10 Hz)

    try:
        while not rospy.is_shutdown():
            plotter.update_plots()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass # Captura interrupção do ROS (Ctrl+C)
    finally:
        plt.ioff() # Desabilita o modo interativo
        plt.show() # Garante que os gráficos fiquem abertos após o término do ROS se desejar
        rospy.loginfo("Nó robot_data_plotter finalizado.")