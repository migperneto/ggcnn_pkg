#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # Importe o CvBridge e CvBridgeError
import cv2 # Importe o OpenCV

class ImageProcessor:
    def __init__(self):
        # 1. Inicializa o nó ROS
        rospy.init_node('camera_display_node', anonymous=True)

        # 2. Cria uma instância do CvBridge
        self.bridge = CvBridge()

        # 3. Define o tópico da câmera. VERIFIQUE SE ESTE TÓPICO CORRESPONDE AO SEU GAZEBO!
        # Você pode ajustar este nome de tópico. Ex: '/my_robot/camera/image_raw'
        self.image_topic = "/camera/color/image_raw" # Tópico padrão para muitas câmeras no Gazebo

        # 4. Cria um Subscriber para o tópico de imagem
        rospy.Subscriber(self.image_topic, Image, self.image_callback)


        rospy.loginfo(f"Nó camera_display_node iniciado. Inscrevendo-se em {self.image_topic}")

    # def image_callback(self, data):
    #     """
    #     Esta função é chamada toda vez que uma nova mensagem de imagem é recebida.
    #     """
    #     try:
    #         # 5. Converte a mensagem ROS Image para uma imagem OpenCV (numpy array)
    #         # 'bgr8' significa 8-bit, 3 canais (BGR)
    #         cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #     except CvBridgeError as e:
    #         rospy.logerr(f"Erro no CvBridge: {e}")
    #         return

    #     # 6. Exibe a imagem usando OpenCV
    #     cv2.imshow("Camera Feed from Gazebo", cv_image)
    #     cv2.waitKey(1) # Espera 1ms por um evento de teclado. 

    def image_callback(self, data):
        try:
            # 5. Converte a mensagem ROS Image para uma imagem OpenCV (numpy array)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Erro no CvBridge: {e}")
            return

        
        # Reduzindo a imagem para 640x480 (opcional, mas pode ajudar na performance)
        cv_image = cv2.resize(cv_image, (440, 280))
        # print(cv_image.shape)
        
        
        # --- NOVAS OPERAÇÕES DE PROCESSAMENTO ---

        # 1. Converter para Grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 2. Converter para HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 3. Exibir as diferentes versões
        cv2.imshow("Original BGR Feed", cv_image)
        cv2.imshow("Grayscale Feed", gray_image)
        cv2.imshow("HSV Feed", hsv_image) # HSV é exibido como uma imagem BGR, mas os valores representam HSV.
  
        # --- FIM DAS NOVAS OPERAÇÕES ---






        # Aplicar um filtro Gaussiano para reduzir ruído
        # (5,5) é o tamanho do kernel (deve ser ímpar). 0 é o desvio padrão (calculado automaticamente)
        blurred_image = cv2.GaussianBlur(cv_image, (5, 5), 0)
        cv2.imshow("Blurred Image", blurred_image)

        # Agora você poderia usar blurred_image para as operações de cor/limiarização
        hsv_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)



        
        imgCanny = cv2.Canny(blurred_image, 100, 200) # Detecta bordas usando Canny
        cv2.imshow("Canny Edges", imgCanny) # Exibe a imagem



        _,th1 = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY) # Limiarização simples
        cv2.imshow("Thresholded Image", th1) # Exibe a imagem limiar
        
        th2 = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2) # Limiarização adaptativa
        cv2.imshow("Adaptive Thresholded Image", th2) # Exibe a imagem lim

        th3 = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2) # Limiarização adaptativa
        cv2.imshow("Adaptive Thresholded Image Mean", th3) # Exibe a imagem




        # --- Filtragem de Cor (exemplo para AZUL) ---
        # Defina os limites de cor em HSV (você terá que ajustar isso para sua cena e luz)
        # Estes são valores aproximados para azul.
        lower_blue = (90, 50, 50)  # Limite inferior de HSV para azul
        upper_blue = (130, 255, 255) # Limite superior de HSV para azul

        # Estes são valores aproximados para vermelho.
        lower_red = (0, 150, 50)  # Limite inferior de HSV para vermelho
        upper_red = (10, 255, 255) # Limite superior de HSV para vermelho

        # Estes são valores aproximados para verde.
        lower_green = (40, 150, 50)  # Limite inferior de HSV para verde
        upper_green = (80, 255, 255) # Limite superior de HSV para verde
        
        # Crie uma máscara para a cor azul usando inRange
        # mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
        mask_blue = cv2.inRange(hsv_image, lower_red, upper_red)
        # mask_blue = cv2.inRange(hsv_image, lower_green, upper_green)

        # Aplique a máscara na imagem original para ver o objeto azul isolado
        res_blue = cv2.bitwise_and(cv_image, cv_image, mask=mask_blue)

        # Exibir a máscara e o resultado filtrado
        cv2.imshow("Blue Mask", mask_blue)
        cv2.imshow("Detected Blue Object", res_blue)

        # --- FIM DA FILTRAGEM DE COR ---





        # --- Operações Morfológicas ---
        # As operações morfológicas são úteis para remover ruídos e melhorar a detecção de objetos.
        # Aqui, vamos aplicar dilatação, erosão, abertura e fechamento na máscara azul.
        # Definir um kernel para as operações morfológicas (geralmente uma matriz de uns)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)) # Kernel 5x5 retangular

        # Aplique Dilatação e Erosão na máscara
        mask_dilated = cv2.dilate(mask_blue, kernel, iterations=1) # Expande 1 vez
        mask_eroded = cv2.erode(mask_blue, kernel, iterations=1) # Encolhe 1 vez

        # Aplique operações de Abertura e Fechamento
        mask_opened = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
        mask_closed = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)

        cv2.imshow("Blue Mask Dilated", mask_dilated)
        cv2.imshow("Blue Mask Eroded", mask_eroded)
        cv2.imshow("Blue Mask Opened", mask_opened)
        cv2.imshow("Blue Mask Closed", mask_closed)

        # --- FIM DAS OPERAÇÕES MORFOLÓGICAS ---






        cv2.waitKey(1) # Espera 1ms para atualizar as janelas    

    def run(self):
        # Mantém o nó rodando até que ele seja parado manualmente (Ctrl+C)
        rospy.spin()
        # Garante que as janelas do OpenCV sejam fechadas ao sair
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        processor = ImageProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nó interrompido.")