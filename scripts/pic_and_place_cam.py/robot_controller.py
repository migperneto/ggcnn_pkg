#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String # Para o controle do gripper, se for um JointTrajectoryController
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # <--- ADICIONE ESTA LINHA
import yaml
import os
import numpy as np # Adicionado import numpy para np.pi

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)

        # Inicializa moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("manipulator") # O nome do seu grupo de planejamento MoveIt!

        # Publicador para visualização no RViz
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Publisher for gripper control (assuming a simple joint trajectory controller for gripper)
        # You'll need to know the joint name for your gripper. Common for UR: 'gripper_joint'
        # Check your URDF or `rostopic list` for gripper topics.
        # If it's a simple command, it might be a Float64.
        # For this example, we'll assume a JointTrajectoryCommand if available
        # Or a custom simple command if configured.
        
        # Check `rostopic list` for `/ur5/gripper_controller/command` type.
        # If it's a JointTrajectory, you'll need a JointTrajectory msg.
        # If it's a simpler Float64 or a custom msg, adjust accordingly.
        # Assuming a simple command for opening/closing for now.
        
        # NOTE: A linha abaixo é um placeholder para um controle de gripper MUITO SIMPLES (String).
        # Para um controle de gripper baseado em JointTrajectory, você precisaria de um JointTrajectory publisher.
        # Se seu gripper usa `/ur5/gripper_controller/command` e ele espera `JointTrajectory`,
        # você deve mudar esta linha para:
        self.gripper_pub = rospy.Publisher('/ur5/gripper_controller/command', JointTrajectory, queue_size=10)
        # E remover a linha `from std_msgs.msg import String` se não for usar String para mais nada.
        # Para o propósito deste exemplo, vamos manter a lógica de String para simplificar,
        # mas você DEVE adaptar para o tipo de mensagem REAL do seu gripper.

        # Carregar parâmetros de poses das caixas
        script_dir = os.path.dirname(__file__)
        config_path = os.path.join(script_dir, '../config/object_params.yaml')
        with open(config_path, 'r') as f:
            self.params = yaml.safe_load(f)

        self.box_positions = self.params['box_positions']
        self.safe_approach_z_offset = self.params['safe_approach_z_offset']
        self.gripper_offset_z = self.params['gripper_offset_z'] # Importar o offset do gripper

        rospy.sleep(1) # Give MoveIt! time to initialize

        # Add table and boxes to the planning scene
        self.add_obstacles_to_scene()

        rospy.loginfo("Robot Controller Node Initialized.")

    def add_obstacles_to_scene(self):
        # Define the table
        table_pose = PoseStamped()
        table_pose.header.frame_id = self.move_group.get_planning_frame()
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = -0.01  # Slightly below base to ensure contact
        table_size = (1.5, 1.0, 0.02) # x, y, z dimensions of the table
        self.scene.add_box("table", table_pose, table_size)

        # Define the boxes (approximate sizes, adjust as needed)
        box_size = (0.2, 0.2, 0.1) # Assuming a simple box for the receptacle

        for color, pos in self.box_positions.items():
            box_pose = PoseStamped()
            box_pose.header.frame_id = self.move_group.get_planning_frame()
            box_pose.pose.position.x = pos['x']
            box_pose.pose.position.y = pos['y']
            box_pose.pose.position.z = pos['z'] + box_size[2] / 2.0 # Center of box
            self.scene.add_box(f"box_{color}", box_pose, box_size)
            rospy.loginfo(f"Added {color} box to scene at {pos}")

        rospy.sleep(2) # Give time for the scene to update

    def go_to_pose(self, pose_stamped):
        self.move_group.set_pose_target(pose_stamped)
        plan = self.move_group.plan()
        
        if not plan[0].joint_trajectory.points:
            rospy.logwarn("Failed to plan trajectory to target pose.")
            return False

        # If a plan is found, execute it
        success = self.move_group.execute(plan[0], wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def open_gripper(self):
        # This part depends heavily on your gripper setup.
        # If it's a JointTrajectoryController:
        traj = JointTrajectory()
        # You MUST get the correct joint name for your gripper from your URDF or `rostopic list`
        # For a Robotiq 2F-85, it might be 'gripper_finger_joint', 'right_finger_joint', etc.
        # Check `rostopic echo /ur5/gripper_controller/state` to see joint names.
        traj.joint_names = ['finger_joint'] # <--- AJUSTE O NOME DA JUNTA DO SEU GRIPPER AQUI!
        point = JointTrajectoryPoint()
        point.positions = [0.0] # Posição de abertura total (ou o valor que abre o seu gripper)
        point.time_from_start = rospy.Duration(1.0) # Tempo para alcançar a posição
        traj.points.append(point)
        self.gripper_pub.publish(traj)
        rospy.sleep(1.0) # Give gripper time to open
        rospy.loginfo("Gripper opened.")

    def close_gripper(self):
        traj = JointTrajectory()
        traj.joint_names = ['finger_joint'] # <--- AJUSTE O NOME DA JUNTA DO SEU GRIPPER AQUI!
        point = JointTrajectoryPoint()
        point.positions = [0.04] # Posição de fechamento (valor que agarra o bloco)
        point.time_from_start = rospy.Duration(1.0)
        traj.points.append(point)
        self.gripper_pub.publish(traj)
        rospy.sleep(1.0) # Give gripper time to close
        rospy.loginfo("Gripper closed.")
    
    def pick(self, object_pose, object_name="object"):
        # The pick action in MoveIt! is quite powerful.
        # It handles pre-grasp, grasp, and post-grasp motions.
        
        # We need to define the grasp configuration relative to the object.
        # This will depend on how your gripper grasps the block.
        # For a simple top-down grasp, the gripper approaches from above the object.

        grasps = []
        grasp = moveit_msgs.msg.Grasp()

        # Pose do gripper em relação ao objeto (ex: gripper alinhado com o eixo Z do objeto, 
        # mas ligeiramente acima para abordagem)
        grasp.grasp_pose.header.frame_id = object_pose.header.frame_id
        grasp.grasp_pose.pose.position.x = object_pose.pose.position.x
        grasp.grasp_pose.pose.position.y = object_pose.pose.position.y
        # A altura exata para preensão. Pode ser a pose do objeto - offset_z
        # Depende de onde a câmera detecta o "centro" do objeto vs. onde o gripper agarra.
        # A pose.position.z do object_detector já é o ponto de preensão.
        grasp.grasp_pose.pose.position.z = object_pose.pose.position.z # + self.gripper_offset_z # Não adicionar offset aqui, já veio do detector
        
        # Orientação do gripper para pegar o objeto (ex: apontando para baixo)
        # (0.707, 0, 0, 0.707) é uma rotação de 90 graus em X (aponta o eixo Z do gripper para baixo)
        # Isso geralmente significa que o eixo Z do EF aponta para baixo, perpendicular à mesa.
        grasp.grasp_pose.pose.orientation.x = 0.707
        grasp.grasp_pose.pose.orientation.y = 0.0
        grasp.grasp_pose.pose.orientation.z = 0.0
        grasp.grasp_pose.pose.orientation.w = 0.707

        # Configuração do movimento pré-grasp (abordagem)
        grasp.pre_grasp_approach.direction.header.frame_id = self.move_group.get_end_effector_link()
        grasp.pre_grasp_approach.direction.vector.z = -1.0  # Mover para baixo no frame do gripper (para abordar o objeto)
        grasp.pre_grasp_approach.min_distance = 0.05
        grasp.pre_grasp_approach.desired_distance = 0.1

        # Configuração do movimento pós-grasp (recuo)
        grasp.post_grasp_retreat.direction.header.frame_id = self.move_group.get_end_effector_link()
        grasp.post_grasp_retreat.direction.vector.z = 1.0  # Mover para cima no frame do gripper (para levantar o objeto)
        grasp.post_grasp_retreat.min_distance = 0.05
        grasp.post_grasp_retreat.desired_distance = 0.1

        # Ação para abrir/fechar o gripper
        grasp.pre_grasp_posture.joint_names = ['finger_joint'] # <--- AJUSTE O NOME DA JUNTA DO SEU GRIPPER AQUI!
        grasp.pre_grasp_posture.points.append(JointTrajectoryPoint(positions=[0.0], time_from_start=rospy.Duration(0.5))) # Aberto

        grasp.grasp_posture.joint_names = ['finger_joint'] # <--- AJUSTE O NOME DA JUNTA DO SEU GRIPPER AQUI!
        grasp.grasp_posture.points.append(JointTrajectoryPoint(positions=[0.04], time_from_start=rospy.Duration(0.5))) # Fechado (valor que agarra)

        grasps.append(grasp)

        rospy.loginfo(f"Attempting to pick {object_name} at {object_pose.pose.position}")
        # Adiciona o objeto à cena de planejamento antes de pegar
        self.scene.add_box(object_name, object_pose, (0.05, 0.05, 0.05)) # Tamanho aproximado do bloco
        
        success = self.move_group.pick(object_name, grasps)
        return success

    def place(self, object_name, target_pose):
        # A ação place do MoveIt!
        # Semelhante ao pick, ele lida com pré e pós-colocação.
        
        places = []
        place = moveit_msgs.msg.PlaceLocation()

        place.place_pose.header.frame_id = target_pose.header.frame_id
        place.place_pose.pose.position.x = target_pose.pose.position.x
        place.place_pose.pose.position.y = target_pose.pose.position.y
        place.place_pose.pose.position.z = target_pose.pose.position.z + self.safe_approach_z_offset # Acima da caixa para soltar
        place.place_pose.pose.orientation = target_pose.pose.orientation # Manter a orientação do gripper

        # Pré-colocação (abordagem)
        place.pre_place_approach.direction.header.frame_id = self.move_group.get_end_effector_link()
        place.pre_place_approach.direction.vector.z = -1.0 # Mover para baixo no frame do gripper
        place.pre_place_approach.min_distance = 0.05
        place.pre_place_approach.desired_distance = 0.1

        # Pós-colocação (recuo)
        place.post_place_retreat.direction.header.frame_id = self.move_group.get_end_effector_link()
        place.post_place_retreat.direction.vector.z = 1.0 # Mover para cima no frame do gripper
        place.post_place_retreat.min_distance = 0.05
        place.post_place_retreat.desired_distance = 0.1

        # Ação para abrir o gripper (liberar o objeto)
        place.post_place_posture.joint_names = ['finger_joint'] # <--- AJUSTE O NOME DA JUNTA DO SEU GRIPPER AQUI!
        place.post_place_posture.points.append(JointTrajectoryPoint(positions=[0.0], time_from_start=rospy.Duration(0.5))) # Aberto

        places.append(place)

        rospy.loginfo(f"Attempting to place {object_name} at {target_pose.pose.position}")
        success = self.move_group.place(object_name, places)
        
        # Após a colocação, remover o objeto da cena de planejamento se for bem-sucedido
        if success:
            self.scene.remove_world_object(object_name)
        return success

    def go_to_home_pose(self):
        # Define uma pose "home" para o robô para evitar colisões
        # Você pode definir um conjunto de juntas ou uma pose cartesiana segura.
        # Exemplo com pose de junta:
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0 # shoulder_pan_joint
        joint_goal[1] = -np.pi/2 # shoulder_lift_joint
        joint_goal[2] = np.pi/2 # elbow_joint
        joint_goal[3] = -np.pi/2 # wrist_1_joint
        joint_goal[4] = -np.pi/2 # wrist_2_joint
        joint_goal[5] = 0 # wrist_3_joint
        
        self.move_group.set_joint_value_target(joint_goal)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.loginfo("Returned to home pose.")
        return success

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("MoveIt! Commander Shutdown.")

if __name__ == '__main__':
    try:
        controller = RobotController()
        
        rospy.spin() # Keep the node alive
        
    except rospy.ROSInterruptException:
        pass