#!/usr/bin/env python3

"""
Este script faz o robô ur5 desviar de obstáculos utilizando a técnica de campos potenciais artificiais adaptativos com algumas melhorias.
Para isso, é utilizado a translaão pelo espaço cartesiano (x,y,z)
1) A força de atração aqui é calculado fazendo a combinação entre o potencial canonico e o parabólico.
2) A força de repulsão foi melhorada por meio da teoria do Campo Potencial Artificial Adaptativo
"""

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from time import sleep
from potential_field3 import PotentialField
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

start_time = time.time()  # Marca o tempo inicial

joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

# APF parameters
k_att = None               # Fator de escala de atração
k_rep = None               # Fator de escala de repulsão (ganho)
rho_0 = None               # Distância mínima de influência
step_size = None           # Tamanho do passo
obstacles = None           # Obstáculos
goal_position = None       # Posição objetivo (Final)
start_position = None      # Posição inicial
d = None

# Variáveis
path = None

def generate_path():
    """Generates a path using the potential field"""
    global potential_field
    global k_att, k_rep, rho_0, step_size, obstacles, goal_position, start_position
    potential_field = PotentialField(k_att, k_rep, rho_0, step_size, d)   # Criando o objeto "potential_field"
    path = potential_field.simulate(start_position, goal_position, obstacles)   # Usando o método simulate da classe PotentialField
    return path

def move_ur5_to_pose(path):
    # Inicializa o MoveIt! Commander e o nó ROS
    moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('ur5_move_to_pose', anonymous=True)

    # Configuração do MoveIt!
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("manipulator") # Cria um objeto MoveGroupCommander para o braço robótico UR5

    #Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    # Define a pose desejada (posição e orientação)
    for i,point in enumerate(path):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = point[0]   # Posição X (metros)
        target_pose.position.y = point[1]   # Posição Y (metros)
        target_pose.position.z = point[2]   # Posição Z (metros)
        target_pose.orientation.w = 1.0 # Mantém a orientação fixa

        # Define o alvo no espaço cartesiano
        move_group.set_pose_target(target_pose)

        # Planeja e executa o movimento
        plan = move_group.go(wait=True)  # Planeja e executa o movimento

        # Libera o alvo após o movimento para evitar que ele permaneça bloqueado
        move_group.stop()  # Garante que o robô pare após o movimento
        move_group.clear_pose_targets() 

        # rospy.sleep(0.01) # Aguarda 0,1 segundos entre os movimentos

        rospy.loginfo(f'Caminho {i}, coordenadas {point}')
        

    # Finaliza a comunicação com o MoveIt!
    moveit_commander.roscpp_shutdown()   # Finaliza MoveIt!
    rospy.loginfo(f"Movimento concluído! Quantidade de passos {i}!")
    rospy.loginfo(f"Coordenada alcançada {point}!")


def get_joint_angles():
    # Criar um objeto MoveGroupCommander para o braço do UR5
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    # Obtém os valores atuais das juntas
    joint_angles = move_group.get_current_joint_values()

    return joint_angles


def save():
   # Dados a serem salvos (trilha no espaço cartesiano)
    path_created = path

    # Nome do arquivo
    nome_arquivo1 = "/home/miguel/miguel_ws/src/trajectory_pkg/scripts/path_created.csv"

    # Abrindo o arquivo em modo de escrita
    with open(nome_arquivo1, mode="w", newline="", encoding="utf-8") as arquivo_csv_1:
        escritor1 = csv.writer(arquivo_csv_1)
        escritor1.writerow(["x", "y", "z"])  # Cabeçalho
        escritor1.writerows(path_created)

    # ************************************************************
    # Dados a serem salvos (histórico da força total)
    his_forces = potential_field.forces_hist() # Uytilizando o método his_forces da classe potential_field
    
    # Nome do arquivo
    nome_arquivo2 = "/home/miguel/miguel_ws/src/trajectory_pkg/scripts/tot_forc_hist_created.csv"

    # Abrindo o arquivo em modo de escrita
    with open(nome_arquivo2, mode="w", newline="", encoding="utf-8") as arquivo_csv_2:
        escritor2 = csv.writer(arquivo_csv_2)
        escritor2.writerow(["x", "y", "z"])  # Cabeçalho
        escritor2.writerows(his_forces[0])        
        
    # ************************************************************
    # Dados a serem salvos (histórico da força de atração)   
    # Nome do arquivo
    nome_arquivo3 = "/home/miguel/miguel_ws/src/trajectory_pkg/scripts/attr_force_hist.csv"

    # Abrindo o arquivo em modo de escrita
    with open(nome_arquivo3, mode="w", newline="", encoding="utf-8") as arquivo_csv_3:
        escritor3 = csv.writer(arquivo_csv_3)
        escritor3.writerow(["x", "y", "z"])  # Cabeçalho
        escritor3.writerows(his_forces[1])        
        
    # ************************************************************
    # Dados a serem salvos (histórico da força de repulsão)   
    # Nome do arquivo
    nome_arquivo4 = "/home/miguel/miguel_ws/src/trajectory_pkg/scripts/rep_force_hist.csv"

    # Abrindo o arquivo em modo de escrita
    with open(nome_arquivo4, mode="w", newline="", encoding="utf-8") as arquivo_csv_4:
        escritor4 = csv.writer(arquivo_csv_4)
        escritor4.writerow(["x", "y", "z"])  # Cabeçalho
        escritor4.writerows(his_forces[2])        
        
    # ************************************************************
    print(f"Arquivos 'path_created.csv', 'tot_forc_hist_created.csv', 'attr_force_hist.csv' e 'rep_force_hist.csv' salvos com sucesso!!!")



def plot_results(path, goal, obstacles, start_position):
    """Plota o caminho, obstáculos e objetivo em 3D."""
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plotando o caminho
    ax.plot(path[:, 0], path[:, 1], path[:, 2], label="Path", linewidth=2, color="blue")

    # Plotando os obstáculos
    for obstacle in obstacles:
        ax.scatter(obstacle[0], obstacle[1], obstacle[2], color='red', marker='o', label="Obstacle", s=100)

    # Plotando o objetivo
    ax.scatter(goal[0], goal[1], goal[2], color='green', marker='x', label="Goal", s=100)

    # Plotando a posição inicial
    ax.scatter(start_position[0], start_position[1], start_position[2], color='black', marker='s', label="Start", s=100)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Artificial Potential Field 3D (Procedural)")
    ax.legend()
    plt.show()


if __name__ == '__main__':
    try:
        print()
        start_time = time.time()  # Marca o tempo inicial

        rospy.init_node('apf_ur5_controller', anonymous=True)
        # APF parameters
        k_att = rospy.get_param('~k_att', 1.0)
        k_rep = rospy.get_param('~k_rep', 0.1)    #0.1
        rho_0 = rospy.get_param('~rho_0', 0.3)   #distância mínima de influência
        step_size = rospy.get_param('~step_size', 0.1)
        obstacles = rospy.get_param('~obstacles', [[-0.4, 0.0, 0.75], [-0.4, 0.191, 0.65]])   # [-0.2, -0.1915, 1.75], [-0.3, 0.191, 1.65]
        goal_position = rospy.get_param('~goal_position', [-0.864, -0.189, 0.202])
        start_position = rospy.get_param('~start_position', [0.096, 0.191, 0.906])
        d = 1*(abs(np.linalg.norm(np.array(goal_position,dtype=float)-np.array(start_position,dtype=float)))) #if distance is less than d, use parabolic potential, otherwise use canonical potential
        
        path = generate_path()   # Obtendo os pontos da trajetória

        save() # Salvando os dados gerados em um arquivo csv "path_created.csv"

        move_ur5_to_pose(path) # Movendo o robô ur5 pelos pontos gerados "path"

        # plot_results(path, goal_position, obstacles, start_position) # Plota o gráfico em 3D Mostrando os pontos deslocados        
 
        end_time = time.time()  # Marca o tempo final
        execution_time = end_time - start_time
        print(f"Tempo de execução: {execution_time:.6f} segundos")

    except rospy.ROSInterruptException:
        pass






