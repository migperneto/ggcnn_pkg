#!/usr/bin/env python

import numpy as np
import moveit_commander

def calcular_jacobiano(q_atual):
    """ Obtém a matriz Jacobiana do manipulador para a configuração atual """
    group = moveit_commander.MoveGroupCommander("manipulator")
    return np.array(group.get_jacobian_matrix(list(q_atual)))  # Conversão de np.array para list

def atualizar_juntas_por_jacobiano(q_atual, forca_cartesiana, dt):
    """ Converte força cartesiana em velocidades das juntas usando J^T """
    J = calcular_jacobiano(q_atual)  # Obtém a Jacobiana
    J_T = J.T  # Transposta da Jacobiana

    torque = np.dot(J_T, forca_cartesiana)  # Multiplicação por J^T
    q_novo = q_atual + torque.flatten() * dt  # Atualiza juntas com integração simples
    
    return q_novo

# Estado inicial das juntas
q_atual = np.array([-0.06657748, -1.4265437,   0.0470515,  -0.03212215,  0.03009033, -1.57003067])

# Força no espaço cartesiano (x, y, z)
forca_cartesiana = np.array([0.1, -0.4, 0.3], dtype=float)
forca_cartesiana_redimen = np.pad(forca_cartesiana, (0, 3), 'constant').reshape(6, 1)

# Calculando os novos ângulos das juntas
q_novo = atualizar_juntas_por_jacobiano(q_atual, forca_cartesiana_redimen, dt=0.5)

# Exibir resultado
print("Novos ângulos das juntas:", list(q_novo))
print("Novos ângulos das juntas: [" + ", ".join(f"{x:.8f}" for x in q_novo) + "]")






