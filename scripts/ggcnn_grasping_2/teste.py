import cv2
import numpy as np

def draw_grasp(img, x, y, theta, width, height=20, color=(0,255,0), thickness=2):
    """
    Desenha um retângulo de preensão na imagem.

    Parâmetros:
        img     : imagem onde desenhar
        x, y    : coordenadas do centro da preensão
        theta   : ângulo da garra (em radianos)
        width   : largura (abertura da garra, em pixels)
        height  : altura fixa do retângulo (espessura da garra, default=20)
        color   : cor do retângulo
        thickness: espessura da linha
    """
    # Metade da largura e altura
    dx = width / 2
    dy = height / 2

    # Coordenadas do retângulo antes da rotação
    rect = np.array([
        [-dx, -dy],
        [ dx, -dy],
        [ dx,  dy],
        [-dx,  dy]
    ])

    # Matriz de rotação
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

    # Aplica rotação
    rect = rect @ R.T

    # Translada para (x, y)
    rect[:, 0] += x
    rect[:, 1] += y

    # Converte para inteiros
    rect = rect.astype(np.int32)

    # Desenha o polígono
    cv2.polylines(img, [rect], isClosed=True, color=color, thickness=thickness)

    return img


if __name__ == "__main__":
    # Exemplo: carregar uma imagem qualquer
    img = np.zeros((480, 640, 3), dtype=np.uint8)

    # Parâmetros de exemplo
    grasp_params = [
        (60, 50, np.pi/6, 80),  # (x, y, θ, largura)
        (400, 300, -np.pi/4, 100),
        (300, 100, np.pi/2, 60)
    ]

    # Desenhar várias preensões
    for (x, y, theta, w) in grasp_params:
        img = draw_grasp(img, x, y, theta, w)

    # Mostrar imagem
    cv2.imshow("Grasps", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
