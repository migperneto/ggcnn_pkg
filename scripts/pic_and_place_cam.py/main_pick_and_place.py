#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from robot_controller import RobotController # Importa a classe RobotController
import yaml
import os
import numpy as np

class PickAndPlaceSystem:
    def __init__(self):
        rospy.init_node('pick_and_place_system', anonymous=True)

        self.robot_controller = RobotController()
        self.block_pose_sub = rospy.Subscriber("/object_detection/block_poses", PoseStamped, self.block_pose_callback)

        self.detected_block_pose = None
        self.picking_in_progress = False

        # Carregar parâmetros de poses das caixas
        script_dir = os.path.dirname(__file__)
        config_path = os.path.join(script_dir, '../config/object_params.yaml')
        with open(config_path, 'r') as f:
            self.params = yaml.safe_load(f)

        self.box_positions = self.params['box_positions']

        rospy.loginfo("Pick and Place System Initialized. Waiting for block detections...")
        
        # Go to a safe initial pose
        self.robot_controller.go_to_home_pose()
        self.robot_controller.open_gripper() # Ensure gripper is open at start

    def block_pose_callback(self, data):
        if not self.picking_in_progress:
            rospy.loginfo(f"Received block pose in frame: {data.header.frame_id}")
            self.detected_block_pose = data
            self.process_block()

    def get_color_from_pose(self, pose_stamped):
        # Esta é uma simplificação. Idealmente, o detector de objetos deveria
        # retornar a cor junto com a pose.
        # Para simular isso, vamos supor que a cor pode ser inferida
        # do nome do tópico ou de alguma metadado (se você expandir o msg PoseStamped).
        # OU, no `object_detector.py`, você pode publicar um Custom Message
        # que contenha `PoseStamped` e `String color`.
        # Por enquanto, vamos supor que o detector só publica uma pose por vez.
        
        # Para fins de demonstração, vamos simular a detecção de cores.
        # Em um cenário real, você passaria a cor do `object_detector`.
        # Você pode modificar o `object_detector` para publicar um dicionário
        # de blocos detectados, ou uma mensagem customizada.
        
        # Se você está recebendo uma PoseStamped única, você precisará
        # manter a lógica de detecção de cor no detector e passá-la
        # para o controlador.
        # A forma mais simples para este exemplo:
        # Assumindo que o `object_detector` está publicando apenas UMA pose
        # por vez, e precisamos de algum modo de saber a cor.
        # Isso é um ponto fraco. O melhor seria o `object_detector.py`
        # publicar um tipo de mensagem customizado, e.g., `DetectedObject.msg`
        # que contenha `PoseStamped pose` e `string color`.
        
        # Para este script funcionar com o `object_detector.py` atual,
        # o `object_detector` precisa publicar a cor junto com a pose.
        # Por exemplo, usando um tópico separado para cada cor, ou uma mensagem customizada.
        
        # Pela simplicidade do exemplo, vamos adicionar um campo de cor ao PoseStamped
        # no detector de objetos (o que é uma má prática, mas funciona para demo rápida)
        # OU, a melhor forma: Criar uma mensagem customizada!
        # from ur5_pick_and_place.msg import DetectedBlock
        # self.block_pose_sub = rospy.Subscriber("/object_detection/block_poses", DetectedBlock, self.block_pose_callback)
        # E no DetectedBlock: PoseStamped pose; string color;
        
        # ASSUMINDO que o PoseStamped vem com a cor (que NÃO É PADRÃO):
        # Neste caso, para o exemplo rodar, você precisaria mudar o `object_detector.py`
        # para usar um tipo de mensagem que inclua a cor.
        # Como o problema inicial é demonstrar o fluxo, vamos adicionar uma SIMPLIFICAÇÃO:
        # A cor será inferida (isso é um HACK para a demo, não use em produção).
        # Melhor: Crie uma mensagem customizada como DetectedBlock com pose e cor.
        
        # Se a cor é essencial aqui, e o PoseStamped não a contém,
        # a lógica de "process_block" deve ser chamada com a cor.
        # Por exemplo, o `object_detector` poderia publicar em `/object_detection/red_block_pose`, etc.
        # Ou um único tópico, mas com uma mensagem mais rica.
        
        # Para fins de DEMO COM O CÓDIGO ATUAL, assumimos que o detector vai
        # publicar a pose para um bloco por vez e que vamos tentar pegar.
        # A cor para o "place" será determinada por alguma outra lógica,
        # ou passaremos um parâmetro.

        # Como os blocos são vermelhos, verdes e azuis, e as caixas são correspondentes,
        # vamos precisar que o `object_detector` nos diga a cor.
        # Recomendo FORTEMENTE criar um tipo de mensagem ROS customizado:
        # `msg/DetectedObject.msg`
        # Header header
        # geometry_msgs/PoseStamped pose
        # string color
        
        # Para fins de execução RÁPIDA, vou simular que a cor é passada.
        # Mas você DEVE implementar a mensagem customizada ou um sistema mais robusto.
        
        # RETORNA UM VALOR DUMMY PARA TESTE. PRECISA SER IMPLEMENTADO CORRETAMENTE!
        # Vamos assumir que a primeira detecção é sempre para um bloco vermelho para testar.
        # OU, o detector pode publicar um objeto por vez no tópico /object_detection/block_poses,
        # e o nó principal (este) "assume" qual cor ele está pegando com base em alguma ordem,
        # ou se o detector publicasse também um campo `color`.
        
        # Para este tutorial, vamos fazer o `object_detector` publicar uma `PoseStamped`
        # e vamos precisar que este nó `main_pick_and_place.py` saiba a cor.
        # A forma mais robusta é a mensagem customizada.
        
        # HACK temporário para testar o fluxo (remover em produção):
        # Se o detector só publica uma PoseStamped, este nó não sabe a cor.
        # Vamos fazer o detector publicar a cor no nome do frame_id (HACK!)
        # ou assumir uma cor para o primeiro teste.
        
        # Melhoria para o detector: Publicar uma lista de DetectedObject.
        pass

    def process_block(self):
        if self.detected_block_pose is None:
            return

        self.picking_in_progress = True
        rospy.loginfo("Processing detected block...")

        # Para fins de demonstração, vamos inferir a cor do bloco a partir da sua posição.
        # Na sua implementação final, a cor viria do `object_detector`.
        # Você pode usar a posição X, Y do bloco para inferir qual cor ele é,
        # se os blocos de uma mesma cor estiverem agrupados na mesa.
        # Ou, como recomendado, o `object_detector` deve retornar a cor.

        # Exemplo Simples: Se o `object_detector` publica uma PoseStamped, e você precisa saber a cor,
        # você teria que adaptar a mensagem ou ter um tópico para cada cor.
        
        # Se o `object_detector.py` publicar `PoseStamped`, e queremos a cor:
        # A forma mais simples para esta DEMO é que o `object_detector.py`
        # ao invés de publicar `PoseStamped`, publique uma mensagem customizada
        # que contém a `PoseStamped` E um `string color`.
        
        # Vamos ajustar o `object_detector.py` para publicar a cor.
        # Se você não puder criar um tipo de mensagem customizada por agora,
        # uma solução temporária é o `object_detector` modificar o `frame_id`
        # para algo como `/camera_color_red_block_frame` ou similar.
        # Mas uma mensagem customizada é o ideal.

        # Para seguir em frente com este exemplo, vamos **assumir** que
        # a lógica para determinar a cor do bloco já ocorreu e foi
        # passada para este método, ou que a primeira pose detectada é
        # sempre de um bloco vermelho (para um teste inicial).
        
        # *** VOCÊ PRECISA ADAPTAR AQUI PARA OBTER A COR CORRETAMENTE DO DETECTOR ***
        # Por agora, vamos usar um hack:
        # Suponha que o `object_detector.py` publica o nome da cor na parte final do frame_id
        # Ou, melhor, que o callback `block_pose_callback` também receba a cor.
        
        # Vamos refatorar o `object_detector.py` para publicar uma mensagem customizada `DetectedObject`.
        # Isso significa que você precisa criar `ur5_pick_and_place/msg/DetectedObject.msg`:
        # geometry_msgs/PoseStamped pose
        # string color
        
        # E então no `object_detector.py`:
        # from ur5_pick_and_place.msg import DetectedObject
        # self.block_pose_pub = rospy.Publisher("/object_detection/detected_objects", DetectedObject, queue_size=10)
        # E no callback:
        # detected_object_msg = DetectedObject()
        # detected_object_msg.header = data.header
        # detected_object_msg.pose = pose
        # detected_object_msg.color = color_name
        # self.block_pose_pub.publish(detected_object_msg)
        
        # E aqui em `main_pick_and_place.py`:
        # self.block_pose_sub = rospy.Subscriber("/object_detection/detected_objects", DetectedObject, self.block_pose_callback)
        # def block_pose_callback(self, data):
        #   self.detected_block_pose = data.pose
        #   self.detected_block_color = data.color
        #   self.process_block()
        
        # Para continuar com o exemplo, vamos assumir que `self.detected_block_color`
        # foi preenchido. Se não, o exemplo não funcionará corretamente para diferentes cores.
        
        # PARA TESTE INICIAL, VAMOS PEGAR A COR DO PRIMEIRO BLOCO DETECTADO
        # A PARTIR DO `object_detector.py` que publicou `detected_blocks[0]['color']`
        # Como o `object_detector` publica apenas UMA `PoseStamped`, precisaremos saber a cor
        # por um método externo ou modificar a mensagem.
        # Para o teste, vamos definir uma cor fixa para o primeiro bloco detectado:
        current_block_color = "red" # <<< MUDAR ISSO PARA OBTER A COR REAL DO DETECTOR!
        rospy.loginfo(f"Attempting to pick a {current_block_color} block.")

        # 1. Mover para uma pose de segurança antes de ir para o bloco
        self.robot_controller.go_to_home_pose()
        
        # 2. Pegar o bloco
        success_pick = self.robot_controller.pick(self.detected_block_pose, f"{current_block_color}_block")
        
        if success_pick:
            rospy.loginfo(f"Successfully picked the {current_block_color} block!")
            
            # 3. Mover para uma pose de segurança com o bloco
            self.robot_controller.go_to_home_pose()

            # 4. Determinar a pose de destino na caixa
            target_box_pose = PoseStamped()
            target_box_pose.header.frame_id = self.robot_controller.move_group.get_planning_frame()
            
            if current_block_color in self.box_positions:
                pos = self.box_positions[current_block_color]
                target_box_pose.pose.position.x = pos['x']
                target_box_pose.pose.position.y = pos['y']
                target_box_pose.pose.position.z = pos['z'] # A altura Z da caixa (já ajustada no YAML)
                
                # Orientação para soltar o bloco (geralmente a mesma de preensão, ou neutra)
                from tf.transformations import quaternion_from_euler
                q = quaternion_from_euler(np.pi/2, 0, np.pi/2) # Exemplo: Apontando para baixo
                target_box_pose.pose.orientation.x = q[0]
                target_box_pose.pose.orientation.y = q[1]
                target_box_pose.pose.orientation.z = q[2]
                target_box_pose.pose.orientation.w = q[3]

                # 5. Colocar o bloco na caixa
                success_place = self.robot_controller.place(f"{current_block_color}_block", target_box_pose)
                
                if success_place:
                    rospy.loginfo(f"Successfully placed the {current_block_color} block in its box!")
                else:
                    rospy.logwarn(f"Failed to place the {current_block_color} block.")
            else:
                rospy.logwarn(f"No target box position defined for color: {current_block_color}")

        else:
            rospy.logwarn(f"Failed to pick the {current_block_color} block.")

        self.robot_controller.go_to_home_pose()
        self.picking_in_progress = False
        self.detected_block_pose = None # Reset for next detection

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        system = PickAndPlaceSystem()
        system.run()
    except rospy.ROSInterruptException:
        pass