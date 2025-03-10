#!/usr/bin/env python3

"""
Este script faz o robô ur5 desviar de obstáculos utilizando a técnica de campos potenciais artificiais clássico.
Para isso, é determinado a trilha no espaço cartesiano (x,y,z), em seguida, determina os ângulos das juntas pela 
cinemática inversa utilizando a biblioteca "trac_ik_python" (link: https://wiki.ros.org/trac_ik_python). 
Para alterar entre o campo potencial básico e o adaptativo, basta alterar a biblioteca. "potential_field3" é a 
que possui o campo adaptativo. (Para eliminar problemas de GNRON (Goals Non-Reachable with Obstacles Nearby (GNRON)) 
presentes nos CPAs clássicos. 

Obs: foi tentado implementar o campo potencial adaptativo, porém, o mesmo apresentou problemas na equação canônica.
Especificamente, na cinemática inversa, por conta de valores "None" que são retornados.

Próximo passo é:
* Implementar o algoritmo Goal Configuration Sampling e Subgoal Selection com a finalidade de elaborar rotas válidas 
alternativas para que o efetuador final alcance o objetivo sem que existam problemas de RLMP (Reacharound Local Minimum Problem
(RLMP)).
* Implementar suavisação de movimento com o polinômio quíntuplo ou outro
"""

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from time import sleep
from inverse_kinematics import inverse_kinematics_ur5
from potential_field3 import PotentialField
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Parâmetros do robô (pode ser obtido do servidor de parâmetros como no seu código original)
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
current_joint_angles = None
path = None

# Publishers
trajectory_publisher = None

def joint_states_callback(msg):
    """Callback para receber o estado das juntas do robô."""
    global current_joint_angles
    current_joint_angles = [msg.position[i] for i in range(len(joint_names))] 

def generate_path():
    """Generates a path using the potential field"""
    global potential_field
    global k_att, k_rep, rho_0, step_size, obstacles, goal_position, start_position
    potential_field = PotentialField(k_att, k_rep, rho_0, step_size, d)   # Criando o objeto "potential_field"
    path = potential_field.simulate(start_position, goal_position, obstacles)   # Usando o método simulate da classe PotentialField
    return path


def convert_cart_cood(path):
    #this function is responsible for obtaining the joint angle from the cartesian coodenation using the inverse kinematics
    global JA, current_joint_angles
    JA = [current_joint_angles]  #list for store the joint angles
    for i,point in enumerate(path):
        #calculates the joints angles with the inverse kinematics
        joint_angles = inverse_kinematics_ur5(point[0], point[1], point[2],JA[i])
        JA.append(joint_angles)  #store the joint angle
        # rospy.loginfo(joint_angles)   #publisher in log  (publica no log os ângulos da juntas0)
        
    return JA


def move_along_path(JA):
    #this function is responsible for move the robot
    global trajectory_publisher, joint_names, pub
    rate = rospy.Rate(10)  # Define the frequency of send the commands

    #criates the mensage of trajectory
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = joint_names

    #define the position for trajectory
    positions = JA

    #add each position as trajectory point with duration time
    for i,pos in enumerate(positions):
        point = JointTrajectoryPoint()
        point.positions = pos
        # point.time_from_start = rospy.Duration(3 * (i + 1))  # Define o tempo em que o ponto deve ser alcançado
        point.time_from_start = rospy.Duration(0.5 * (i + 1))
    
        trajectory_msg.points.append(point)
    
    #publisher the trajectory in topic
    pub.publish(trajectory_msg)
    rospy.loginfo("Mensagem de trajetória completa enviada para o UR5")


def loop():
    global current_joint_angles, path, start_position, JA
    if current_joint_angles is not None:
        path = generate_path()  #generate the path in cartesian coordinates (has all the coordinates)
        # rospy.loginfo(path)  #publisher in log the path (publica no log a trilha das coordenadas catesianas)
        #convert using the inverse kinematics
        joint_angles = convert_cart_cood(path) 
        #move the robot using the joint angle
        move_along_path(JA)  
        rospy.loginfo("Path following complete.")

def save():
   # Dados a serem salvos
    path_created_0 = path
    joint_angles_created_0 = JA

    # Nome do arquivo
    nome_arquivo1 = "/home/miguel/miguel_ws/src/trajectory_pkg/scripts/path_created_0.csv"
    nome_arquivo2 = "/home/miguel/miguel_ws/src/trajectory_pkg/scripts/joint_angles_created_0.csv"

    # Abrindo o arquivo em modo de escrita
    with open(nome_arquivo1, mode="w", newline="", encoding="utf-8") as arquivo_csv_1:
        escritor1 = csv.writer(arquivo_csv_1)
        escritor1.writerows(path_created_0)

    with open(nome_arquivo2, mode="w", newline="", encoding="utf-8") as arquivo_csv_2:
        escritor2 = csv.writer(arquivo_csv_2)
        escritor2.writerows(joint_angles_created_0)

    print(f"Arquivos 'path_created_0.csv' e 'joint_angles_created_0.csv' salvos com sucesso!!!")


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
        rospy.init_node('apf_ur5_controller', anonymous=True)
        # APF parameters
        k_att = rospy.get_param('~k_att', 1.0)
        k_rep = rospy.get_param('~k_rep', 0.1)   
        rho_0 = rospy.get_param('~rho_0', 0.3)   #distância mínima de influência
        step_size = rospy.get_param('~step_size', 0.1)
        obstacles = rospy.get_param('~obstacles', [[-0.4, 0.0, 0.75], [-0.4, 0.191, 0.65]])
        goal_position = rospy.get_param('~goal_position', [-0.864, -0.189, 0.202])
        start_position = rospy.get_param('~start_position', [0.096, 0.191, 0.906])
        d = 1*(abs(np.linalg.norm(np.array(goal_position,dtype=float)-np.array(start_position,dtype=float)))) #if distance is less than d, use parabolic potential, otherwise use canonical potential
        
        # Publishers
        pub = trajectory_publisher = rospy.Publisher("/ur5/eff_joint_traj_controller/command", JointTrajectory, queue_size=10)
        # Subscriber
        rospy.Subscriber("/ur5/joint_states", JointState, joint_states_callback)  #obtair the joint angle initial (current_joint_angles)

        # Espera até receber uma mensagem sobre os ângulos das juntas
        while current_joint_angles is None:
           rospy.loginfo("Waiting for joint states")
           rospy.sleep(0.1)

        rospy.loginfo("Starting APF path following.")
        
        loop() #function main.

        print('Report:')
        print(f'The number of iterations was {potential_field.iterations}!!!')

        save()

        # plot_results(path, goal_position, obstacles, start_position)
        

    except rospy.ROSInterruptException:
        pass








