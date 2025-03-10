#!/usr/bin/env python3
import numpy as np

def dh_transform(theta, d, a, alpha):
    """Calcula a matriz de transformação homogênea com base nos parâmetros DH."""
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,             np.sin(alpha),                  np.cos(alpha),                 d],
        [0,             0,                              0,                             1]
    ])

def forward_kinematics(joint_angles, dh_params):
    """Calcula a posição cartesiana do TCP dado os ângulos das juntas e os parâmetros DH."""
    T = np.eye(4)  # Matriz identidade 4x4

    for i, joint_angle in enumerate(joint_angles):
        theta, d, a, alpha = dh_params[i]
        T_i = dh_transform(joint_angle + theta, d, a, alpha)
        T = np.dot(T, T_i)  # Multiplicação das matrizes de transformação

    position = T[:3, 3]  # Extrai a posição (x, y, z) da matriz de transformação final
    return position

if __name__ == "__main__":
    # Parâmetros de Denavit-Hartenberg do robô UR5
    # Formato: [theta, d, a, alpha]
    dh_params = [
        [0, 0.089159, 0, np.pi / 2],
        [0, 0, -0.425, 0],
        [0, 0, -0.39225, 0],
        [0, 0.10915, 0, np.pi / 2],
        [0, 0.09465, 0, -np.pi / 2],
        [0, 0.0823, 0, 0]
    ]

    # Entrada dos ângulos das juntas em radianos
    print("Insira os ângulos das juntas em radianos:")
    joint_angles = []
    for i in range(6):
        angle = float(input(f"Junta {i+1}: "))
        joint_angles.append(angle)

    # Calcula a posição cartesiana
    position = forward_kinematics(joint_angles, dh_params)

    # Exibe os resultados
    print("\nPosição no espaço cartesiano:")
    print(f"x: {position[0]:.4f} m")
    print(f"y: {position[1]:.4f} m")
    print(f"z: {position[2]:.4f} m")
