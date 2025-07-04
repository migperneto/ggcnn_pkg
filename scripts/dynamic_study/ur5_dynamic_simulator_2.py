#!/usr/bin/env python3

import rospy
from urdf_parser_py.urdf import URDF
import PyKDL
import kdl_parser_py.urdf as kdl_parser
import numpy as np

def load_chain():
    robot = URDF.from_parameter_server()
    success, kdl_tree = kdl_parser.treeFromUrdfModel(robot)
    if not success:
        raise RuntimeError("Erro ao gerar KDL Tree")
    chain = kdl_tree.getChain("base_link", "tool0")
    return chain

def simulate_dynamics(chain, torques):
    # Estado inicial
    q = PyKDL.JntArray(chain.getNrOfJoints())
    q_dot = PyKDL.JntArray(chain.getNrOfJoints())
    q_dotdot = PyKDL.JntArray(chain.getNrOfJoints())
    tau = PyKDL.JntArray(chain.getNrOfJoints())
    
    for i in range(chain.getNrOfJoints()):
        tau[i] = torques[i]

    dyn_solver = PyKDL.ChainDynParam(chain, PyKDL.Vector(0, 0, -9.81))
    M = PyKDL.JntSpaceInertiaMatrix(chain.getNrOfJoints())
    C = PyKDL.JntArray(chain.getNrOfJoints())
    G = PyKDL.JntArray(chain.getNrOfJoints())

    dyn_solver.JntToMass(q, M)
    dyn_solver.JntToCoriolis(q, q_dot, C)
    dyn_solver.JntToGravity(q, G)

    print("Matriz de Inércia:\n", M)
    # print("Matriz de Coriolis e Centrífugos:\n", C)
    print("Torques de Gravidade:\n", [G[i] for i in range(G.rows())])

    return M, G

def kdl_inertia_to_numpy(inertia_matrix):
    n = inertia_matrix.rows()
    M = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            M[i, j] = inertia_matrix[i, j]
    return M

if __name__ == "__main__":
    rospy.init_node("ur5_dynamics_simulation")
    chain = load_chain()
    torques = [50, 30, 10, 5, 5, 5]  # Exemplo de torque
    M, G = simulate_dynamics(chain, torques)

    M = kdl_inertia_to_numpy(M)
    print("Matriz de Inércia:\n", M)
