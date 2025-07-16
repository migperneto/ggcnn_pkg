#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

def move_ur5_smooth():
    rospy.init_node('ur5_stepwise_motion', anonymous=True)
    pub = rospy.Publisher('/ur5/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)  # Espera o publisher se conectar

    # Lista de posições
    positions = [
        [0, -1.57, 0, -1.57, 0, 1.57],  # Posicione inicial.
        [0.0006755823210884415, -1.5762144678802201, -0.9863883901350183, -1.9275579209404494, 1.7701385269807446, -1.5596449591229034], # posição 1 de preensão
        [-0.1898686902512159, -2.5201492577827675, -0.4890622974277683, -1.790991671395683, 1.58526403320257, -1.6579733598110629],  # posição do ur5 para a caixa azul
        [0.0006755823210884415, -1.5762144678802201, -0.9863883901350183, -1.9275579209404494, 1.7701385269807446, -1.5596449591229034], # posição 1 de preensão
        [0.12738779362844088, -2.462544106412323, -0.6847554150672153, -1.568369542910041, 1.6261684603545428, -1.3932726955579806],  # posição do ur5 para a caixa verde
        [0.0006755823210884415, -1.5762144678802201, -0.9863883901350183, -1.9275579209404494, 1.7701385269807446, -1.5596449591229034], # posição 1 de preensão
        [0.439039108951115, -2.4681369733767333, -0.6956734922676429, -1.5727009906932032, 1.626084585349334, -1.3933260072862605],  # posição do ur5 para a caixa vermelha
       [0.0006755823210884415, -1.5762144678802201, -0.9863883901350183, -1.9275579209404494, 1.7701385269807446, -1.5596449591229034], # posição 1 de preensão
        [0, -1.57, 0, -1.57, 0, 1.57]  # Posicione inicial.
    ]

    duration_per_point = 3.0  # segundos para atingir cada ponto
    delay_after_motion = 2.0  # segundos de pausa após atingir o ponto

    for idx, pos in enumerate(positions):
        rospy.loginfo(f"Movendo para a posição {idx + 1}")

        # Cria mensagem de trajetória com 1 ponto
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", 
            "elbow_joint", "wrist_1_joint", 
            "wrist_2_joint", "wrist_3_joint"
        ]

        point = JointTrajectoryPoint()
        point.positions = pos
        point.time_from_start = rospy.Duration(duration_per_point)

        traj_msg.points.append(point)

        pub.publish(traj_msg)
        rospy.sleep(duration_per_point + delay_after_motion)

    rospy.loginfo("Movimentos finalizados.")

if __name__ == '__main__':
    try:
        move_ur5_smooth()
    except rospy.ROSInterruptException:
        pass






# #!/usr/bin/env python3

# import rospy
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# import time
# import pandas as pd


# def move_ur5():
#     # Inicialize o nó ROS
#     rospy.init_node('ur5_trajectory_control', anonymous=True)
#     # Configura o publisher para o tópico de controle do UR5
#     pub = rospy.Publisher('/ur5/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    
        
#     # Crie a mensagem de trajetória
#     trajectory_msg = JointTrajectory()
#     trajectory_msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", 
#                                   "elbow_joint", "wrist_1_joint", 
#                                   "wrist_2_joint", "wrist_3_joint"]
    
#     # Carregar dados do arquivo CSV 
#     # positions = pd.read_csv('scripts/joint_angles_created_0.csv')
#     # positions = positions.values.tolist()
    
#     # Defina as posições para o movimento
#     positions = [
#         [0, -1.57, 0, -1.57, 0, 1.57],  # Posicione inicial.
#         [0.0006755823210884415, -1.5762144678802201, -0.9863883901350183, -1.9275579209404494, 1.7701385269807446, -1.5596449591229034], # posição 1 de preensão
#         [-0.1898686902512159, -2.5201492577827675, -0.4890622974277683, -1.790991671395683, 1.58526403320257, -1.6579733598110629],  # posição do ur5 para a caixa azul
#         [0.0006755823210884415, -1.5762144678802201, -0.9863883901350183, -1.9275579209404494, 1.7701385269807446, -1.5596449591229034], # posição 1 de preensão
#         [0.12738779362844088, -2.462544106412323, -0.6847554150672153, -1.568369542910041, 1.6261684603545428, -1.3932726955579806],  # posição do ur5 para a caixa verde
#         [0.0006755823210884415, -1.5762144678802201, -0.9863883901350183, -1.9275579209404494, 1.7701385269807446, -1.5596449591229034], # posição 1 de preensão
#         [0.439039108951115, -2.4681369733767333, -0.6956734922676429, -1.5727009906932032, 1.626084585349334, -1.3933260072862605],  # posição do ur5 para a caixa vermelha
#        [0.0006755823210884415, -1.5762144678802201, -0.9863883901350183, -1.9275579209404494, 1.7701385269807446, -1.5596449591229034], # posição 1 de preensão
#         [0, -1.57, 0, -1.57, 0, 1.57]  # Posicione inicial.
#     ]
   
    
#     # Adicione cada posição como um ponto de trajetória com tempo de duração
#     for i, pos in enumerate(positions):
#         point = JointTrajectoryPoint()
#         point.positions = pos
#         point.time_from_start = rospy.Duration(3 * (i + 1))  # Define o tempo em que o ponto deve ser alcançado
#         trajectory_msg.points.append(point)
    
#     # Publique a trajetória no tópico
#     rospy.sleep(1)  # Pequeno delay para garantir que o nó está ativo
#     pub.publish(trajectory_msg)
#     rospy.loginfo("Mensagem de trajetória completa enviada para o UR5")

# if __name__ == '__main__':
#     try:
#         move_ur5()
#     except rospy.ROSInterruptException:
#         pass
