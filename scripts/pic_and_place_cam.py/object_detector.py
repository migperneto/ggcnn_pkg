#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from cv_bridge.core import CvBridgeError # <--- ADICIONE ESTA LINHA
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
import sensor_msgs.point_cloud2 as pc2
import yaml
import os
import sys

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)

        self.bridge = CvBridge()
        self.color_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_callback)
        self.point_cloud_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.point_cloud_callback)

        self.block_pose_pub = rospy.Publisher("/object_detection/block_poses", PoseStamped, queue_size=10)

        self.cv_image = None
        self.points_data = None
        self.processed_frame_id = ""

        # Carregar parâmetros de cor e posições das caixas
        script_dir = os.path.dirname(__file__)
        config_path = os.path.join(script_dir, '../../config/object_params.yaml')
        with open(config_path, 'r') as f:
            self.params = yaml.safe_load(f)

        self.color_ranges = self.params['color_ranges']
        self.gripper_offset_z = self.params['gripper_offset_z']

        rospy.loginfo("Object Detector Node Initialized.")

    def color_image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.processed_frame_id = data.header.frame_id
        except CvBridgeError as e:
            rospy.logerr(e)

    def point_cloud_callback(self, data):
        # Apenas processa a nuvem de pontos se tivermos uma imagem colorida correspondente
        if self.cv_image is not None and data.header.frame_id == self.processed_frame_id:
            self.points_data = data
            self.detect_and_publish_objects()
            self.cv_image = None # Limpa a imagem para esperar o próximo par sincronizado

    def detect_and_publish_objects(self):
        if self.cv_image is None or self.points_data is None:
            return

        hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        detected_blocks = []

        for color_name, color_range in self.color_ranges.items():
            lower_bound = np.array(color_range['lower'])
            upper_bound = np.array(color_range['upper'])

            mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

            # Para o vermelho, que pode ter dois intervalos HSV
            if color_name == 'red' and 'lower2' in color_range:
                lower_bound2 = np.array(color_range['lower2'])
                upper_bound2 = np.array(color_range['upper2'])
                mask2 = cv2.inRange(hsv_image, lower_bound2, upper_bound2)
                mask = cv2.bitwise_or(mask, mask2)

            # Opcional: Operações morfológicas para limpar a máscara
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Ajuste o limiar de área conforme o tamanho dos seus blocos
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        # Obter coordenadas 3D da nuvem de pontos
                        # Iteramos sobre uma pequena janela ao redor do centroide para robustez
                        # É importante que a nuvem de pontos e a imagem estejam alinhadas!
                        points_list = []
                        for u in range(max(0, cx-5), min(self.points_data.width, cx+5)):
                            for v in range(max(0, cy-5), min(self.points_data.height, cy+5)):
                                # Opcional: verificar se o pixel está dentro do contorno
                                # if cv2.pointPolygonTest(contour, (u,v), False) >= 0:
                                try:
                                    point_data = list(pc2.read_points(self.points_data, field_names=("x", "y", "z"), skip_nans=True, uvs=[[u,v]]))
                                    if len(point_data) > 0:
                                        points_list.append(point_data[0])
                                except Exception as e:
                                    # rosnod error if point is invalid
                                    pass

                        if points_list:
                            # Calcular a média das coordenadas 3D
                            x_avg = np.mean([p[0] for p in points_list])
                            y_avg = np.mean([p[1] for p in points_list])
                            z_avg = np.mean([p[2] for p in points_list])

                            if not (np.isnan(x_avg) or np.isnan(y_avg) or np.isnan(z_avg)):
                                # Ajustar a pose Z para a altura de preensão (considerando o offset do gripper)
                                # A altura Z da câmera é a altura do objeto, o gripper precisa ir um pouco abaixo.
                                # Assumimos que o Z da câmera é positivo para "frente" da câmera.
                                # O Z do frame base do robô é geralmente para cima.
                                # Você pode precisar ajustar o sinal de Z e Y dependendo da orientação da sua câmera e do frame do robô.
                                
                                # A transformação entre o frame da câmera (/camera_link ou /camera_depth_optical_frame)
                                # e o frame base do robô é crucial aqui.
                                # Por simplicidade inicial, vamos assumir que as coordenadas da nuvem de pontos já estão
                                # aproximadamente no frame base do robô, o que é comum em simulações se o TF estiver configurado.
                                
                                # Se o Z obtido da nuvem de pontos for a altura acima do chão no frame da câmera,
                                # você precisará transformar isso para o frame do robô.
                                # Para a simulação, o Gazebo geralmente publica o `PointCloud2` no frame da câmera (`camera_depth_optical_frame`).
                                # O MoveIt! opera no frame base do robô (`base_link`).
                                
                                # Exemplo de publicação da pose:
                                # A PoseStamped é sempre no frame de referência da nuvem de pontos (data.header.frame_id).
                                # Para o MoveIt!, você precisará transformar essa pose para o frame do robô.
                                # Isso é geralmente feito pelo próprio MoveIt! se a árvore TF estiver correta, ou manualmente.

                                pose = PoseStamped()
                                pose.header.frame_id = self.points_data.header.frame_id # Provavelmente 'camera_depth_optical_frame'
                                pose.header.stamp = rospy.Time.now()
                                pose.pose.position.x = x_avg
                                pose.pose.position.y = y_avg
                                pose.pose.position.z = z_avg - self.gripper_offset_z # Ajuste para a ponta do gripper

                                # Orientação: Para pegar blocos simples, uma orientação padrão é suficiente.
                                # MoveIt! geralmente pode resolver isso. Uma orientação "neutra" é (0,0,0,1) que é apontando para cima (Z)
                                # ou (0.707, 0, 0, 0.707) para apontar X para baixo (em relação ao frame do robô).
                                # Você pode precisar girar o gripper.
                                pose.pose.orientation.w = 1.0 # Exemplo: Sem rotação
                                # Se o gripper precisa estar perpendicular à mesa:
                                # from tf.transformations import quaternion_from_euler
                                # q = quaternion_from_euler(np.pi/2, 0, 0) # Rotação de 90 graus em X para apontar para baixo
                                # pose.pose.orientation.x = q[0]
                                # pose.pose.orientation.y = q[1]
                                # pose.pose.orientation.z = q[2]
                                # pose.pose.orientation.w = q[3]

                                detected_blocks.append({'color': color_name, 'pose': pose})

                                # Desenhar contorno e centroide na imagem (para depuração)
                                cv2.drawContours(self.cv_image, [contour], -1, (0, 255, 0), 2)
                                cv2.circle(self.cv_image, (cx, cy), 5, (0, 0, 255), -1)
                                cv2.putText(self.cv_image, f"{color_name} ({x_avg:.2f},{y_avg:.2f},{z_avg:.2f})", (cx - 50, cy - 20),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Exibir a imagem processada (para depuração)
        # cv2.imshow("Processed Image", self.cv_image)
        # cv2.waitKey(1)
        
        # Publica as poses dos blocos (vamos publicar uma por vez, a mais próxima ou baseada em alguma lógica)
        # Para simplificar, vamos publicar a primeira pose detectada.
        # No main_pick_and_place.py, você vai iterar sobre os blocos.
        if detected_blocks:
            # Para o exemplo, vamos publicar apenas o primeiro bloco detectado.
            # No cenário real, você pode querer classificar por distância ou alguma prioridade.
            self.block_pose_pub.publish(detected_blocks[0]['pose'])
            rospy.loginfo(f"Detected and published {detected_blocks[0]['color']} block at X:{detected_blocks[0]['pose'].pose.position.x:.2f}, Y:{detected_blocks[0]['pose'].pose.position.y:.2f}, Z:{detected_blocks[0]['pose'].pose.position.z:.2f}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass