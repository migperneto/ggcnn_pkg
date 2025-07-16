#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState

# Parâmetros de Denavit-Hartenberg (DH) do robô UR5
# Formato: [theta, d, a, alpha]
# theta: ângulo rotacional (variável para juntas rotacionais)
# d: deslocamento ao longo do eixo z
# a: comprimento do elo ao longo do eixo x
# alpha: ângulo entre os eixos z adjacentes

joint_angles = []

DH_PARAMS = [
    [0, 0.089159, 0        , np.pi / 2 ],
    [0, 0       , -0.425   , 0         ],
    [0, 0       , -0.39225 , 0         ],
    [0, 0.10915 , 0        , np.pi / 2 ],
    [0, 0.09465 , 0        , -np.pi / 2],
    [0, 0.0823  , 0        , 0         ]
]

def dh_transform(theta, d, a, alpha):
    """Calcula a matriz de transformação homogênea com base nos parâmetros DH."""
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,             np.sin(alpha),                  np.cos(alpha),                 d],
        [0,             0,                              0,                             1]
    ])

def forward_kinematics(joint_angles):
    """Calcula a cinemática direta do robô dado os ângulos das juntas."""
    T = np.eye(4)  # Matriz identidade 4x4

    for i, joint_angle in enumerate(joint_angles):
        theta, d, a, alpha = DH_PARAMS[i]
        T_i = dh_transform(joint_angle + theta, d, a, alpha)
        T = np.dot(T, T_i)  # Multiplicação das matrizes de transformação

    return [-T[:3,3][0],-T[:3,3][1],T[:3,3][2]]

def joint_state_callback(msg):
    """Callback para processar o estado das juntas e calcular a posição cartesiana."""
    joint_angles = msg.position  # Ângulos das juntas em radianos

    # Calcula a cinemática direta
    tcp_transform = forward_kinematics(joint_angles)

    # Extrai a posição e orientação da matriz de transformação
    position = tcp_transform  # Elementos de translação (x, y, z)
    orientation = tcp_transform[:3, :3]  # Matriz de rotação

    rospy.loginfo("Posição TCP no Espaço Cartesiano:")
    rospy.loginfo(f"  x: {position[0]:.3f}")
    rospy.loginfo(f"  y: {position[1]:.3f}")
    rospy.loginfo(f"  z: {position[2]:.3f}")
    rospy.loginfo("Matriz de Rotação:")
    rospy.loginfo(f"{orientation}")  

    rospy.loginfo(f'ângulo das juntas:')
    rospy.loginfo(joint_angles)

def ur5_cartesian_listener():
    rospy.init_node('ur5_cartesian_listener', anonymous=True)

    # Subscrição ao tópico de estados das juntas
    rospy.Subscriber('/ur5/joint_states', JointState, joint_state_callback)

    rospy.loginfo("Subscriber para o TCP do UR5 iniciado. Aguardando dados...")
    # rospy.spin()

# if __name__ == '__main__':
#     try:
#         ur5_cartesian_listener()
#     except rospy.ROSInterruptException:
#         pass

# ur5_cartesian_listener()



# joint_angles = [0.001502366387020615, -1.5699573339977295, -0.0032821566105987188, 0.0018254296884796517, -0.00017268888238231028, -1.5699690207809667]
# Matriz_transf = forward_kinematics(joint_angles)
# pos = Matriz_transf

# print(pos)


