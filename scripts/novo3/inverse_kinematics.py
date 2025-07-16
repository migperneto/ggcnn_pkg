#!/usr/bin/env python3

import numpy as np
from trac_ik_python.trac_ik import IK

def inverse_kinematics_ur5(x, y, z, current_joint_angles):
    """
    Resolve a cinemática inversa do robô UR5 para um ponto no espaço cartesiano.
    
    Parâmetros:
    - x, y, z: Posição desejada no espaço cartesiano para o efetuador final.
    
    Retorna:
    - Lista de soluções para os ângulos articulares [theta1, theta2, ..., theta6].
    """
    # Criando o solver IK para o UR5 (verifique os nomes das frames no seu SRDF/URDF)
    ik_solver = IK("base_link", "tool0")  # Substitua pelos nomes corretos, caso seja necessário

    # Valores iniciais aproximados das juntas (ponto de partida para a solução)
    q_init = current_joint_angles
  
    # Coordenadas desejadas no espaço cartesiano + orientação (quaternion w, x, y, z)
    desired_pose = [x, y, z, 0, 0, 0, 1]  # [x, y, z, w, qx, qy, qz]

    # Resolver a cinemática inversa
    joint_angles = list(ik_solver.get_ik(q_init, *desired_pose))

    return joint_angles




current_joint_angles = [0.001, -1.58, -1.005, -1.937, 1.770, -1.56]
inv = inverse_kinematics_ur5(-0.427131, 0.003185, 0.4, current_joint_angles)
print("Ângulos das juntas:", inv)

