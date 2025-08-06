#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
import tf.transformations
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
import tf.transformations


# Variáveis globais para armazenar a última pose recebida
latest_position = None
latest_orientation = None

# Callback chamado sempre que chega uma nova mensagem no tópico
def pose_callback(msg):
    global latest_position, latest_orientation

    # Pegando posição e orientação
    latest_position = msg.pose.position
    # latest_orientation = msg.pose.orientation   # [3.098, 0.025, 3.121] # roll, pitch, yaw (Euler)
    latest_orientation = [3.098, 0.025, 3.121] # roll, pitch, yaw (Euler)
   
    rospy.loginfo("Recebido Pose:")
    rospy.loginfo("  Posição: x=%.2f, y=%.2f, z=%.2f", 
                  latest_position.x, latest_position.y, latest_position.z)
    rospy.loginfo("  Orientação: roll=%.2f, pitch=%.2f, yaw=%.2f", 
                  latest_orientation[0], latest_orientation[1], latest_orientation[2])



# ********** PLANEJANDO MOVIMENTO NO ESPAÇO CARTESIANO ***************

def move_ur5_cartesiano():

    global latest_position, latest_orientation
    # Cria um objeto MoveGroupCommander para o braço robótico UR5
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    #Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = latest_position.x   # Posição X (metros)
    target_pose.position.y = latest_position.y   # Posição Y (metros)
    target_pose.position.z = latest_position.z + 0.19  # Posição Z (metros)
    
    # Define a orientação do end-effector usando ângulos de Euler
    roll = latest_orientation[0] # valor para roll
    pitch = latest_orientation[1] # valor para pitch (aproximadamente 90 graus)
    yaw = latest_orientation[2] # valor para yaw

    # Converte os ângulos de Euler para um quatérnion
    # A função quaternion_from_euler espera (roll, pitch, yaw) em radianos
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)    

    # Orientação definida como um quaternion (w,x,y,z)
    target_pose.orientation.x = quaternion[0]
    target_pose.orientation.y = quaternion[1]
    target_pose.orientation.z = quaternion[2]
    target_pose.orientation.w = quaternion[3]  # Define a orientação do end-effector

    # Define o alvo no espaço cartesiano
    move_group.set_pose_target(target_pose)

    # Planeja e executa o movimento
    plan = move_group.go(wait=True)  # Planeja e executa diretamente

    # Libera o alvo após o movimento para evitar que ele permaneça bloqueado
    move_group.stop()
    move_group.clear_pose_targets()

    # Finaliza a comunicação com o MoveIt!
    # moveit_commander.roscpp_shutdown()   

# ********** FIM DO PLANEJANDO MOVIMENTO NO ESPAÇO CARTESIANO ***************


# ********** PLANEJANDO MOVIMENTO NO ESPAÇO DAS JUNTAS ***************
def move_ur5_joints(position_joint):
    print(position_joint)

    # Cria um objeto MoveGroupCommander para o braço robótico UR5
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    #Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

    # We get the joint values from the group and change some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = position_joint[2]
    joint_goal[1] = position_joint[1]
    joint_goal[2] = position_joint[0]
    joint_goal[3] = position_joint[3]
    joint_goal[4] = position_joint[4]
    joint_goal[5] = position_joint[5]

    move_group.go(joint_goal, wait=True)
    move_group.stop()

    # Finaliza a comunicação com o MoveIt!
    # moveit_commander.roscpp_shutdown()
    # rospy.loginfo("Movimento Executado!")

# ********** FIM DO PLANEJANDO MOVIMENTO NO ESPAÇO DAS JUNTAS ***************


# ********** FUNÇÃO PARA COMANDAR AS GARRAS DO ROBÔ UR5 ***************
def send_gripper_command(joint_name, target_position, duration=1.0):
    # Inicializa o nó ROS
    if not rospy.core.is_initialized():
        rospy.init_node('ur5_gripper_commander', anonymous=True)

    # Cria um publicador para o tópico de comando da garra
    pub = rospy.Publisher('/ur5/gripper_controller/command', JointTrajectory, queue_size=10)

    # Espera um pouco para o publicador se conectar ao tópico
    rospy.sleep(0.5)

    # Cria a mensagem JointTrajectory
    gripper_command = JointTrajectory()
    gripper_command.header.stamp = rospy.Time.now() # Carimbo de tempo atual
    gripper_command.joint_names = [joint_name] # Nome da junta da garra

    # Cria um ponto de trajetória
    point = JointTrajectoryPoint()
    point.positions = [target_position] # Posição alvo da junta
    point.time_from_start = rospy.Duration(duration) # Tempo para alcançar a posição

    gripper_command.points.append(point) # Adiciona o ponto à trajetória

    # Publica a mensagem
    rospy.loginfo(f"Enviando comando para a garra: {joint_name} para a posição {target_position} em {duration} segundos.")
    pub.publish(gripper_command)

    # Espera a duração para o movimento ser concluído
    rospy.sleep(duration + 0.5) 



def open_gripper(OPEN_POSITION):
    """
    Comanda a garra para abrir.
    """
    GRIPPER_JOINT_NAME = 'robotiq_85_left_knuckle_joint' 

    # OPEN_POSITION = 0.00 

    send_gripper_command(GRIPPER_JOINT_NAME, OPEN_POSITION)
    rospy.loginfo("Comando de abertura da garra enviado.")




def close_gripper(CLOSE_POSITION): 
    """
    Comanda a garra para fechar.
    """
    GRIPPER_JOINT_NAME = 'robotiq_85_left_knuckle_joint' 

    # CLOSE_POSITION = 0.46     # 0.798

    send_gripper_command(GRIPPER_JOINT_NAME, CLOSE_POSITION)
    rospy.loginfo("Comando de fechamento da garra enviado.")


# ******** FIM DA FUNÇÃO PARA COMANDAR AS GARRAS DO ROBÔ UR5 **********


if '__main__' == __name__:
    try:

        # Inicializa o MoveIt! Commander e o nó ROS
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur5_move_to_grasp', anonymous=True)
        rospy.Subscriber("/detected_object_pose_baselink", PoseStamped, pose_callback)


        # # Order of blocks: blue 1, green 1, red 1, blue 2, green 2, red 2
        # # ordem_blocks = ['blue 1', 'green 1', 'red 1', 'blue 2', 'green 2', 'red 2']   # Completo
        # ordem_blocks = ['blue 1', 'green 1', 'red 1']   # Metade
        # # Pose of blocks x, y , z and orientation roll, pitch, yaw
        # position_blocks = [[-0.427131, 0.003185, 0.19],[-0.560503, -0.221624, 0.19], [-0.571972, 0.200934, 0.19], 
        #                 [-0.424412, -0.143902, 0.19], [-0.441868, 0.201144, 0.19], [-0.564329, 0.006137, 0.19]]   
        # orientation_blocks = [-3.090, 0.00, 3.121] # roll, pitch, yaw

        # # Boxes: blue, green, red
        # position_box = [[-0.8, 0.240476, 0.3], [-0.8, -0.009524, 0.3], [-0.8, -0.259524, 0.3]]  # x, y , z
        # orientation_blocks = [-3.090, 0.015, 3.121] # roll, pitch, yaw

        # # Position for intermediate grasp
        # position_intermediate_grasp = [-0.446, 0.109, 0.4]  # x, y , z
        # orientation_intermediate_grasp = [3.140, -0.190, 3.131] # roll, pitch, yaw

        # Position of grip 1
        position_grip_1 = [-1.0054704413213962, -1.5801199410581797, 0.00089008570573057, -1.9367237375089363, 1.5699304075994247, -1.5595961735668054]    # posição 1 de preensão
        position_grip_2 = [-1.83, -1.453, 0.00, -1.26, 1.57, -1.56]

        # Posição vertical
        position_vertical = [0, -1.57, 0, -1.57, 0, 1.57]

        rospy.loginfo("Starting System Operation")
        move_ur5_joints(position_vertical) # Move to vertical position        

        rospy.loginfo("Text gripper: Closing and opening gripper!!!!")
        close_gripper(0.798)  # Close gripper
        # rospy.sleep(0.7)  # Wait for gripper to close
        open_gripper(0.0)  # Open gripper
        # rospy.sleep(0.7)  # Wait for gripper to open

        # Moving the robot to the grip position
        rospy.loginfo("Moving to grip position!!!")
        move_ur5_joints(position_grip_2)  # Move to Position of grip 1
        # rospy.sleep(0.2)


        # for n, block in enumerate(ordem_blocks):

        # Moving to block desired
        rospy.loginfo("Moving to block block!!!")
        move_ur5_cartesiano()
        # rospy.sleep(0.2)
        close_gripper(0.46)  # Close gripper
        # rospy.sleep(0.2) 


        # Moving to intermediate position_grip_1
        rospy.loginfo("Moving to grip position!!!")
        move_ur5_joints(position_grip_2)  # Move to Position of grip 1
        # rospy.sleep(0.2) 

        rospy.loginfo("Moving to blue box!!!")
        move_ur5_cartesiano()
        open_gripper(0.0)  # Open gripper
        # rospy.sleep(0.3)


        # # Moving to intermediate grasp
        # rospy.loginfo("Moving to intermediate grasp!!!")
        # move_ur5_cartesiano()  # Move to intermediate grasp
        # # rospy.sleep(0.3)


        # # Moving to the box
        # if (block == 'blue 1' or block == 'blue 2'):
        #     rospy.loginfo("Moving to blue box!!!")
        #     move_ur5_cartesiano()
        #     open_gripper(0.0)  # Open gripper
        #     # rospy.sleep(0.3)

        #     elif (block == 'green 1' or block == 'green 2'):
        #         rospy.loginfo("Moving to green box!!!")
        #         move_ur5_cartesiano(val=position_box[1], euler=orientation_blocks)
        #         open_gripper(0.0)  # Open gripper
        #         # rospy.sleep(0.3)

        #     elif (block == 'red 1' or block == 'red 2'):
        #         rospy.loginfo("Moving to red box!!!")
        #         move_ur5_cartesiano(val=position_box[2], euler=orientation_blocks)
        #         open_gripper(0.0)  # Open gripper
        #         # rospy.sleep(0.3)

        #     # Moving to intermediate grasp
        #     rospy.loginfo("Moving to intermediate grasp!!!")
        #     move_ur5_cartesiano(val=position_intermediate_grasp, euler=orientation_intermediate_grasp)  # Move to intermediate grasp
        #     # rospy.sleep(0.3)

        move_ur5_joints(position_vertical) # Move to vertical position

        # Finaliza a comunicação com o MoveIt!
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("Movimento Executado!")



    except rospy.ROSInterruptException:
        pass


























# #******MODELO EXEMPLO PARA RECEBER POSE DE UM OBJETO DETECTADO********

# #!/usr/bin/env python

# import rospy
# from geometry_msgs.msg import PoseStamped

# # Variáveis globais para armazenar a última pose recebida
# latest_position = None
# latest_orientation = None

# # Callback chamado sempre que chega uma nova mensagem no tópico
# def pose_callback(msg):
#     global latest_position, latest_orientation

#     # Pegando posição e orientação
#     latest_position = msg.pose.position
#     latest_orientation = msg.pose.orientation   # [3.098, 0.025, 3.121] # roll, pitch, yaw (Euler)

#     rospy.loginfo("Recebido Pose:")
#     rospy.loginfo("  Posição: x=%.2f, y=%.2f, z=%.2f", 
#                   latest_position.x, latest_position.y, latest_position.z)
#     rospy.loginfo("  Orientação: x=%.2f, y=%.2f, z=%.2f, w=%.2f", 
#                   latest_orientation.x, latest_orientation.y, 
#                   latest_orientation.z, latest_orientation.w)

# def usa_pose_em_outra_funcao():
#     if latest_position is None or latest_orientation is None:
#         rospy.logwarn("Ainda não há dados de pose recebidos.")
#         return

#     # Aqui você pode usar a posição e orientação como quiser
#     rospy.loginfo("Usando a última pose armazenada:")
#     rospy.loginfo("  Posição: (%.2f, %.2f, %.2f)" % 
#                   (latest_position.x, latest_position.y, latest_position.z))
#     rospy.loginfo("  Orientação (quaternion): (%.2f, %.2f, %.2f, %.2f)" % 
#                   (latest_orientation.x, latest_orientation.y, 
#                    latest_orientation.z, latest_orientation.w))

# def main():
#     rospy.init_node('pose_listener_node')

#     rospy.Subscriber("/detected_object_pose_baselink", PoseStamped, pose_callback)

#     rate = rospy.Rate(1)  # 1 Hz

#     while not rospy.is_shutdown():
#         usa_pose_em_outra_funcao()
#         rate.sleep()

# if __name__ == "__main__":
#     main()
