#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo 
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
import tf



class ImageProcessor:
    def __init__(self):
        rospy.init_node('camera_display_node', anonymous=True)
        self.bridge = CvBridge()

        # Tópicos
        self.rgb_image_topic = "/camera/color/image_raw"
        self.depth_image_topic = "/camera/depth/image_raw"
        self.camera_info_topic = "/camera/color/camera_info"    # "/camera/depth/camera_info"
        self.object_pose_publisher = rospy.Publisher('/detected_object_pose_baselink', PoseStamped, queue_size=10)
    
        # Subscribers
        self.rgb_sub = rospy.Subscriber(self.rgb_image_topic, Image, self.rgb_image_callback)
        self.depth_sub = rospy.Subscriber(self.depth_image_topic, Image, self.depth_image_callback)
        self.info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)

        # Variáveis para armazenar as imagens e informações da câmera
        self.cv_image = None
        self.depth_image = None
        self.camera_intrinsics = None # Matriz K da CameraInfo
        self.camera_frame_id = "" # O frame da câmera, útil para a pose

        # --- Listener de TF ---
        self.tf_listener = tf.TransformListener()

        rospy.loginfo(f"Nó camera_processor_node iniciado. Inscrevendo-se em {self.rgb_image_topic}, {self.depth_image_topic}, {self.camera_info_topic}")




    def camera_info_callback(self, data):
        """
        Recebe as informações intrínsecas da câmera e as armazena.
        Esta callback é chamada apenas uma vez ou quando a info muda.
        """
        if self.camera_intrinsics is None: # Apenas armazena uma vez
            self.camera_intrinsics = np.array(data.K).reshape((3, 3))
            self.camera_frame_id = data.header.frame_id
            rospy.loginfo(f"Camera Intrinsics (K) recebidos:\n{self.camera_intrinsics}")
            rospy.loginfo(f"Camera Frame ID: {self.camera_frame_id}")
            # Desinscreva-se após receber a info
            self.info_sub.unregister()


    def rgb_image_callback(self, data):
        """
        Recebe a imagem RGB e a armazena. O processamento principal será aqui.
        """
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Erro no CvBridge (RGB): {e}")

        # Chama a função de processamento principal sempre que uma nova imagem RGB chega
        self.process_images()



    def depth_image_callback(self, data):
        """
        Recebe a imagem de profundidade e a armazena.
        Câmeras de profundidade publicam em 16UC1 (milímetros) ou 32FC1 (metros)
        """
        try:
            # Câmera RealSense no Gazebo, profundidade em float32 (metros)
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            rospy.logerr(f"Erro no CvBridge (Depth): {e}")



    def process_images(self):
        """
        Função principal de processamento de imagem, chamada quando uma nova imagem RGB chega.
        """
        if self.cv_image is None or self.depth_image is None or self.camera_intrinsics is None:
            # Espera até que todas as informações necessárias estejam disponíveis
            rospy.loginfo("Aguardando imagens RGB, de profundidade e CameraInfo...")
            return

        # --- Processamento da Imagem ---
        # Reduzindo a imagem para 640x480 (opcional, mas pode ajudar na performance)
        cv_image = cv2.resize(self.cv_image, (440, 280))

        # Aplicando um filtro Gaussiano para suavizar a imagem
        blurred_image = cv2.GaussianBlur(self.cv_image, (7, 7), 0)
        hsv_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)

        # Definindo os limites de cor em HSV (ajuste conforme necessário)

        # # Estes são valores aproximados para azul.
        # lower_blue = (90, 50, 50)  # Limite inferior de HSV para azul
        # upper_blue = (130, 255, 255) # Limite superior de HSV para azul

        # # Estes são valores aproximados para vermelho.
        # lower_red = (0, 150, 50)  # Limite inferior de HSV para vermelho
        # upper_red = (10, 255, 255) # Limite superior de HSV para vermelho

        # # Estes são valores aproximados para verde.
        # lower_green = (40, 150, 50)  # Limite inferior de HSV para verde
        # upper_green = (80, 255, 255) # Limite superior de HSV para verde


        lower_bound = np.array([90, 50, 50])
        upper_bound = np.array([130, 255, 255])
        # Criando a máscara
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

        # Aplicando operações morfológicas para limpar a máscara
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Copiando a imagem original para desenhar os objetos detectados
        detected_objects_image = self.cv_image.copy()
        mask_copy = mask.copy()
        # Encontrando contornos na máscara
        contours, hierarchy = cv2.findContours(mask_copy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000: # Ajuste este valor conforme o tamanho do seu objeto
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx_pixel = int(M["m10"] / M["m00"])
                    cy_pixel = int(M["m01"] / M["m00"])

                    # Desenha contorno e centróide na imagem de exibição
                    cv2.drawContours(detected_objects_image, [contour], -1, (0, 255, 0), 2)
                    cv2.circle(detected_objects_image, (cx_pixel, cy_pixel), 5, (0, 0, 0), -1)
                    # cv2.putText(detected_objects_image, f"({cx_pixel}, {cy_pixel})", (cx_pixel + 10, cy_pixel + 10),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    # --- CONVERTER PIXEL 2D PARA PONTO 3D ---
                    try:
                        # Obter o valor de profundidade do pixel do centróide
                        # Certifique-se de que cx_pixel e cy_pixel estão dentro dos limites da imagem de profundidade
                        if 0 <= cy_pixel < self.depth_image.shape[0] and 0 <= cx_pixel < self.depth_image.shape[1]:
                            depth_value = self.depth_image[cy_pixel, cx_pixel] # Valor em metros (se 32FC1) ou mm (se 16UC1)

                            if np.isnan(depth_value) or depth_value == 0:
                                rospy.logwarn("Profundidade inválida (NaN ou 0) no pixel do centróide. Pulando objeto.")
                                continue

                            # Converte de milímetros para metros se necessário (16UC1)
                            Z_camera = float(depth_value)/1000  

                            # Obter os parâmetros intrínsecos da câmera (fx, fy, cx_k, cy_k)
                            fx = self.camera_intrinsics[0, 0]
                            fy = self.camera_intrinsics[1, 1]
                            cx_k = self.camera_intrinsics[0, 2] # cx da matriz K
                            cy_k = self.camera_intrinsics[1, 2] # cy da matriz K

                            # Desprojetar para coordenadas 3D no frame da câmera
                            X_camera = (cx_pixel - cx_k) * Z_camera / fx
                            Y_camera = (cy_pixel - cy_k) * Z_camera / fy

                            # rospy.loginfo(f"Objeto detectado! Centróide 2D: ({cx_pixel}, {cy_pixel})")
                            # rospy.loginfo(f"Profundidade (Z_camera): {Z_camera:.4f} m")
                            # rospy.loginfo(f"Posição 3D (frame da câmera): X={X_camera:.4f} m, Y={Y_camera:.4f} m, Z={Z_camera:.4f} m")

                            # --- Publicar a pose do objeto como PointStamped para o TF ---
                            # Criamos um PointStamped no frame da câmera
                            point_in_camera_frame = PointStamped()
                            point_in_camera_frame.header.frame_id = self.camera_frame_id
                            point_in_camera_frame.header.stamp = rospy.Time(0) # Usar o último dado disponível
                            point_in_camera_frame.point.x = X_camera
                            point_in_camera_frame.point.y = Y_camera
                            point_in_camera_frame.point.z = Z_camera

                            # Esperar e fazer a transformação para o frame base do robô
                            target_frame = "base_link" 
                            try:
                                # Espera pela transformação entre os frames
                                self.tf_listener.waitForTransform(target_frame, self.camera_frame_id, rospy.Time.now(), rospy.Duration(4.0))

                                # Transforma o ponto do frame da câmera para o frame base
                                point_in_base_frame = self.tf_listener.transformPoint(target_frame, point_in_camera_frame)
                               
                                # --- Publicar a pose final em um tópico ROS para o MoveIt usar ---
                                final_pose = PoseStamped()
                                final_pose.header.frame_id = target_frame
                                final_pose.header.stamp = rospy.Time.now()
                                final_pose.pose.position.x = point_in_base_frame.point.x
                                final_pose.pose.position.y = point_in_base_frame.point.y
                                final_pose.pose.position.z = point_in_base_frame.point.z
                                final_pose.pose.orientation.w = 1.0 
                                
                                self.object_pose_publisher.publish(final_pose)

                                # Log para depuração
                                rospy.loginfo(f"Objeto detectado! Posição no frame da câmera (x,y,z): ({X_camera:.4f}, {Y_camera:.4f}, {Z_camera:.4f})")
                                rospy.loginfo(f"Posição no frame {target_frame} (x,y,z): ({final_pose.pose.position.x:.4f}, {final_pose.pose.position.y:.4f}, {final_pose.pose.position.z:.4f})")
                                
                                # Saia do loop após encontrar e processar o primeiro objeto
                                break

                            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                                rospy.logerr(f"Erro de TF: {e}")
                                continue

                    except Exception as e:
                        rospy.logerr(f"Erro ao desprojetar ponto 3D: {e}")
                        continue

        # Exibir as imagens
        cv2.imshow("Original BGR Feed", self.cv_image)
        cv2.imshow("Color Mask (Binary)", mask)
        cv2.imshow("Detected Object (with Contours & Centroid)", detected_objects_image)

        cv2.waitKey(1)


    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        processor = ImageProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nó interrompido.")
    finally:
        # Garante que as janelas sejam fechadas mesmo se houver um erro
        cv2.destroyAllWindows()