#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
import tf.transformations
# import garra
# import position_grip
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

def move_ur5_cartesiano(val, euler):

    # Cria um objeto MoveGroupCommander para o braço robótico UR5
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    #Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = val[0]   # Posição X (metros)
    target_pose.position.y = val[1]   # Posição Y (metros)
    target_pose.position.z = val[2]   # Posição Z (metros)
    
    # Define a orientação do end-effector usando ângulos de Euler
    
    roll = euler[0] # valor para roll
    pitch = euler[1] # valor para pitch (aproximadamente 90 graus)
    yaw = euler[2] # valor para yaw

    # Converte os ângulos de Euler para um quatérnion
    # A função quaternion_from_euler espera (roll,8 pitch, yaw) em radianos
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



# if '__main__' == __name__:
#     try:

#         # Inicializa o MoveIt! Commander e o nó ROS
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node('ur5_move_to_grasp', anonymous=True)



#         # Order of blocks: blue 1, green 1, red 1, blue 2, green 2, red 2
#         ordem_blocks = ['blue 1', 'green 1', 'red 1', 'blue 2', 'green 2', 'red 2']
#         # Pose of blocks x, y , z and orientation roll, pitch, yaw
#         position_blocks = [[-2.223227844138586, -1.5975670721842938, 0.24097119454288318, -0.8946125301880326, 1.6203262088188568, -1.309466599377533],
#                            [-1.746992614188538, -1.9751005729257116, 0.5527334528128778, -0.9809564743647519, 1.6136790141372437, -0.9967395752014188], 
#                            [-1.723978492744779, -1.984274524817467, -0.16405711953390867, -1.0285922240788121, 1.6226710835743008, -1.7150313329560323], 
#                            [-2.176307257601474, -1.64605645507637, 0.5650710218568333, -0.8768140537458144, 1.613276417315924, -0.9850618067448176], 
#                            [-2.066942476049843, -1.7309810352830262, -0.20918170150656135, -0.9398578581853263, 1.6204613976673965, -1.7599683921116718], 
#                            [-1.8550829804563618, -1.897237648176656, 0.17638380360182015, -0.9674630430678004, 1.6205337928463486, -1.3729405150193355]]   
    
#         # Boxes: blue, green, red
#         position_box = [[-0.5008817206553751, -2.5309001530019994, -0.16569502370928824, -1.7241717276560955, 1.6187000888291863, -1.7159241497353737], 
#                         [-0.7769139205662485, -2.3825929379726034, 0.14319825340289594, -1.5791315915651962, 1.6248232139083019, -1.4069595987148054], 
#                         [-0.4817734355037153, -2.5451971393955075, 0.43868511752112305, -1.69724901491902, 1.6236019178759236, -1.1100579804398247]]  

#         # Position for intermediate grasp
#         # position_intermediate_grasp = [-1.5701470457789934, -1.4516880522137985, 0.000253752333129853, -1.5184446505798688, 1.570947077181061, -1.559754733121042]
#         position_intermediate_grasp = [-1.70, -1.44, 0.00, -1.39, 1.57, -1.56]  # posição mais próximo dos blocos


#         # Position of grip 1
#         position_grip_1 = [-1.0054704413213962, -1.5801199410581797, 0.00089008570573057, -1.9367237375089363, 1.5699304075994247, -1.5595961735668054]    # posição 1 de preensão

#         # Posição vertical
#         position_vertical = [0, -1.57, 0, -1.57, 0, 1.57]

#         rospy.loginfo("Starting System Operation")
#         move_ur5_joints(position_vertical) # Move to vertical position

#         rospy.loginfo("Text gripper: Closing and opening gripper!!!!")
#         close_gripper(0.798)  # Close gripper
#         # rospy.sleep(0.1)  # Wait for gripper to close
#         open_gripper(0.0)  # Open gripper
#         # rospy.sleep(0.1)  # Wait for gripper to open

#         # Moving the robot to the grip position
#         rospy.loginfo("Moving to grip position!!!")
#         move_ur5_joints(position_grip_1)  # Move to Position of grip 1
#         # rospy.sleep(0.1)


#         for n, block in enumerate(ordem_blocks):

#             # Moving to block
#             print(f"Moving to {block} block!!!")
#             move_ur5_joints(position_blocks[n])
#             # rospy.sleep(0.1)
#             close_gripper(0.46)  # Close gripper
#             # rospy.sleep(0.1)   

            
#             # Moving to intermediate grasp
#             rospy.loginfo("Moving to intermediate grasp!!!")
#             move_ur5_joints(position_intermediate_grasp)  # Move to intermediate grasp
#             # rospy.sleep(0.1)


#             # Moving to the box
#             if (block == 'blue 1' or block == 'blue 2'):
#                 rospy.loginfo("Moving to blue box!!!")
#                 move_ur5_joints(position_box[0])
#                 open_gripper(0.0)  # Open gripper
#                 # rospy.sleep(0.2)

#             elif (block == 'green 1' or block == 'green 2'):
#                 rospy.loginfo("Moving to green box!!!")
#                 move_ur5_joints(position_box[1])
#                 open_gripper(0.0)  # Open gripper
#                 # rospy.sleep(0.2)

#             elif (block == 'red 1' or block == 'red 2'):
#                 rospy.loginfo("Moving to red box!!!")
#                 move_ur5_joints(position_box[2])
#                 open_gripper(0.0)  # Open gripper
#                 # rospy.sleep(0.2)

#             # Moving to intermediate grasp
#             rospy.loginfo("Moving to intermediate grasp!!!")
#             move_ur5_joints(position_intermediate_grasp)  # Move to intermediate grasp
#             # rospy.sleep(0.1)

        
#         # move_ur5_joints(position_grip_1)  # Move to Position of grip 1  
#         # rospy.sleep(0.1) 


#         move_ur5_joints(position_vertical) # Move to vertical position

#         # Finaliza a comunicação com o MoveIt!
#         moveit_commander.roscpp_shutdown()
#         rospy.loginfo("Movimento Executado!")



#     except rospy.ROSInterruptException:
#         pass

























if '__main__' == __name__:
    try:

        # Inicializa o MoveIt! Commander e o nó ROS
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur5_move_to_grasp', anonymous=True)



        # Order of blocks: blue 1, green 1, red 1, blue 2, green 2, red 2
        ordem_blocks = ['blue 1', 'green 1', 'red 1', 'blue 2', 'green 2', 'red 2']   # Completo
        # ordem_blocks = ['blue 1', 'green 1', 'red 1']   # Metade
        # Pose of blocks x, y , z and orientation roll, pitch, yaw
        position_blocks = [[-0.427131, 0.003185, 0.19],[-0.560503, -0.221624, 0.19], [-0.571972, 0.200934, 0.19], 
                        [-0.424412, -0.143902, 0.19], [-0.441868, 0.201144, 0.19], [-0.564329, 0.006137, 0.19]]   
        orientation_blocks = [-3.090, 0.00, 3.121] # roll, pitch, yaw

        # Boxes: blue, green, red
        position_box = [[-0.8, 0.240476, 0.3], [-0.8, -0.009524, 0.3], [-0.8, -0.259524, 0.3]]  # x, y , z
        orientation_blocks = [-3.090, 0.015, 3.121] # roll, pitch, yaw

        # Position for intermediate grasp
        position_intermediate_grasp = [-0.446, 0.109, 0.4]  # x, y , z
        orientation_intermediate_grasp = [3.140, -0.190, 3.131] # roll, pitch, yaw

        # Position of grip 1
        position_grip_1 = [-1.0054704413213962, -1.5801199410581797, 0.00089008570573057, -1.9367237375089363, 1.5699304075994247, -1.5595961735668054]    # posição 1 de preensão


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
        move_ur5_joints(position_grip_1)  # Move to Position of grip 1
        # rospy.sleep(0.2)


        for n, block in enumerate(ordem_blocks):

            # Moving to block desired
            rospy.loginfo(f"Moving to {block} block!!!")
            move_ur5_cartesiano(val=position_blocks[n], euler=orientation_blocks)
            # rospy.sleep(0.2)
            close_gripper(0.46)  # Close gripper
            # rospy.sleep(0.2)   


            # Moving to intermediate grasp
            rospy.loginfo("Moving to intermediate grasp!!!")
            move_ur5_cartesiano(val=position_intermediate_grasp, euler=orientation_intermediate_grasp)  # Move to intermediate grasp
            # rospy.sleep(0.3)


            # Moving to the box
            if (block == 'blue 1' or block == 'blue 2'):
                rospy.loginfo("Moving to blue box!!!")
                move_ur5_cartesiano(val=position_box[0], euler=orientation_blocks)
                open_gripper(0.0)  # Open gripper
                # rospy.sleep(0.3)

            elif (block == 'green 1' or block == 'green 2'):
                rospy.loginfo("Moving to green box!!!")
                move_ur5_cartesiano(val=position_box[1], euler=orientation_blocks)
                open_gripper(0.0)  # Open gripper
                # rospy.sleep(0.3)

            elif (block == 'red 1' or block == 'red 2'):
                rospy.loginfo("Moving to red box!!!")
                move_ur5_cartesiano(val=position_box[2], euler=orientation_blocks)
                open_gripper(0.0)  # Open gripper
                # rospy.sleep(0.3)

            # Moving to intermediate grasp
            rospy.loginfo("Moving to intermediate grasp!!!")
            move_ur5_cartesiano(val=position_intermediate_grasp, euler=orientation_intermediate_grasp)  # Move to intermediate grasp
            # rospy.sleep(0.3)

        move_ur5_joints(position_vertical) # Move to vertical position

        # Finaliza a comunicação com o MoveIt!
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("Movimento Executado!")



    except rospy.ROSInterruptException:
        pass