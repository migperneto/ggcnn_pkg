#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # Importe CvBridgeError para tratamento de erros
import cv2
import numpy as np # Vamos precisar de numpy para o kernel morfológico

# --- Funções e Variáveis Globais para Trackbars ---
# (Poderíamos colocar dentro da classe, mas para simplicidade inicial, globais funcionam)

# Nome da janela onde as trackbars aparecerão
TRACKBAR_WINDOW_NAME = "HSV Color Tuner"

def nothing(x):
    # Esta é uma função de callback vazia que cv2.createTrackbar exige.
    # Ela não precisa fazer nada para o nosso caso de uso.
    pass

# Valores iniciais para os limites HSV.
# Você pode definir valores razoáveis para começar, por exemplo, para azul:
# H_min=90, S_min=50, V_min=50
# H_max=130, S_max=255, V_max=255

# Ou comece com a faixa mais ampla para explorar:
initial_h_min = 0
initial_s_min = 0
initial_v_min = 0
initial_h_max = 179 # Hue vai de 0 a 179 no OpenCV
initial_s_max = 255
initial_v_max = 255

class ImageProcessor:
    def __init__(self):
        rospy.init_node('camera_display_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_topic = "/camera/color/image_raw" # Lembre-se de ajustar este tópico se necessário
        rospy.Subscriber(self.image_topic, Image, self.image_callback)

        rospy.loginfo(f"Nó camera_display_node iniciado. Inscrevendo-se em {self.image_topic}")

        # --- CRIAR A JANELA E AS TRACKBARS AQUI ---
        cv2.namedWindow(TRACKBAR_WINDOW_NAME)
        cv2.createTrackbar("H_min", TRACKBAR_WINDOW_NAME, initial_h_min, 179, nothing)
        cv2.createTrackbar("S_min", TRACKBAR_WINDOW_NAME, initial_s_min, 255, nothing)
        cv2.createTrackbar("V_min", TRACKBAR_WINDOW_NAME, initial_v_min, 255, nothing)
        cv2.createTrackbar("H_max", TRACKBAR_WINDOW_NAME, initial_h_max, 179, nothing)
        cv2.createTrackbar("S_max", TRACKBAR_WINDOW_NAME, initial_s_max, 255, nothing)
        cv2.createTrackbar("V_max", TRACKBAR_WINDOW_NAME, initial_v_max, 255, nothing)
        # --- FIM DA CRIAÇÃO DE TRACKBARS ---


    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Erro no CvBridge: {e}")
            return

        # --- PROCESSAMENTO DA IMAGEM ---

        # 1. Aplicar Gaussiano para suavizar (bom para remover ruído antes de HSV)
        blurred_image = cv2.GaussianBlur(cv_image, (7, 7), 0) # Kernel maior para mais suavização

        # 2. Converter para HSV
        hsv_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)

        # 3. Obter os valores atuais das Trackbars
        # Estes valores são lidos a cada frame, então você verá as mudanças em tempo real!
        h_min = cv2.getTrackbarPos("H_min", TRACKBAR_WINDOW_NAME)
        s_min = cv2.getTrackbarPos("S_min", TRACKBAR_WINDOW_NAME)
        v_min = cv2.getTrackbarPos("V_min", TRACKBAR_WINDOW_NAME)
        h_max = cv2.getTrackbarPos("H_max", TRACKBAR_WINDOW_NAME)
        s_max = cv2.getTrackbarPos("S_max", TRACKBAR_WINDOW_NAME)
        v_max = cv2.getTrackbarPos("V_max", TRACKBAR_WINDOW_NAME)

        # lower_bound = np.array([h_min, s_min, v_min])
        # upper_bound = np.array([h_max, s_max, v_max])
        lower_bound = np.array([0, 150, 50])
        upper_bound = np.array([10, 255, 255])

        # 4. Criar a máscara usando cv2.inRange()
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

        # 5. Aplicar Operações Morfológicas na máscara (Ex: Abertura para remover pequenos ruídos)
        # Um kernel pequeno (e.g., 3x3) geralmente é bom
        kernel = np.ones((3,3), np.uint8) # Cria um kernel de 3x3 com todos os valores 1
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) # Abertura: Erosão seguida de Dilatação
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # Fechamento: Dilatação seguida de Erosão (opcional, para preencher buracos)


        # 6. Aplicar a máscara na imagem original para visualizar o objeto detectado
        # res = resultado
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)








        # --- NOVAS OPERAÇÕES: ENCONTRAR E ANALISAR CONTORNOS ---

        # Crie uma cópia da máscara porque findContours a modifica
        mask_copy = mask.copy()

        # Encontre os contornos na máscara
        # cv2.findContours retorna 2 ou 3 valores dependendo da versão do OpenCV
        # Para Python 3 e OpenCV 4+, geralmente retorna (contours, hierarchy)
        contours, hierarchy = cv2.findContours(mask_copy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Desenhe todos os contornos encontrados (opcional, útil para depuração)
        # c_image = cv_image.copy() # Opcional: desenhar em uma cópia da imagem original
        # cv2.drawContours(c_image, contours, -1, (0, 255, 0), 2) # Desenha todos em verde, espessura 2

        detected_objects_image = cv_image.copy() # Imagem para desenhar os objetos detectados

        # Itere sobre os contornos encontrados
        for contour in contours:
            # Calcule a área do contorno
            area = cv2.contourArea(contour)

            # Filtre contornos muito pequenos (provavelmente ruído)
            # Você precisa ajustar este valor '1000' com base no tamanho dos seus objetos
            if area > 1000:
                # Calcule os momentos do contorno
                M = cv2.moments(contour)

                # Verifique se o momento 'm00' não é zero para evitar divisão por zero
                if M["m00"] != 0:
                    # Calcule o centróide (cx, cy)
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Desenhe o contorno e o centróide na imagem original
                    cv2.drawContours(detected_objects_image, [contour], -1, (0, 255, 0), 2) # Verde
                    cv2.circle(detected_objects_image, (cx, cy), 5, (0, 0, 255), -1) # Centróide em vermelho
                    cv2.putText(detected_objects_image, f"({cx}, {cy})", (cx + 10, cy + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) # Coordenadas em branco

                    rospy.loginfo(f"Objeto detectado! Centróide: ({cx}, {cy}), Área: {area}")

        # --- FIM DAS OPERAÇÕES DE CONTORNO ---






        # --- EXIBIÇÃO DAS IMAGENS ---
        cv2.imshow("Original BGR Feed", cv_image)
        cv2.imshow("HSV Feed", hsv_image) # Exibido como BGR, mas representa HSV
        cv2.imshow("Color Mask (Binary)", mask) # A máscara é o que você vai calibrar
        cv2.imshow("Detected Object", res) # O objeto realçado
        cv2.imshow("Detected Object (with Contours & Centroid)", detected_objects_image) # Nova janela


        # --- MANTÉM JANELAS ATUALIZADAS ---
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