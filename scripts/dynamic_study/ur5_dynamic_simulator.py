#!/usr/bin/env python3

import rospy
from urdf_parser_py.urdf import URDF
import PyKDL
import kdl_parser_py.urdf as kdl_parser 
import numpy as np
import time
import matplotlib.pyplot as plt

# --- Constants for Simulation ---
DT = 0.01  # Time step for integration (seconds)
SIM_DURATION = 1.0 # Total simulation duration (seconds)

# --- 1. Load the URDF Model and Create KDL Chain ---
def load_ur5_kdl_chain():
    rospy.loginfo("Attempting to load URDF from ROS parameter server...")
    try:
        robot = URDF.from_parameter_server()
        if robot is None:
            raise RuntimeError("URDF model could not be loaded from parameter server.")
        rospy.loginfo("URDF model loaded successfully.")
    except Exception as e:
        rospy.logerr(f"Failed to load URDF from parameter server: {e}")
        rospy.logerr("Ensure a UR5 launch file (e.g., ur_description/launch/ur5_rviz.launch or ur_gazebo/launch/ur5_bringup.launch) is running.")
        return None

    success, kdl_tree = kdl_parser.treeFromUrdfModel(robot)
    if not success:
        rospy.logerr("Failed to build KDL Tree from URDF model.")
        return None
    rospy.loginfo("KDL Tree built successfully.")

    ur5_chain = kdl_tree.getChain("base_link", "tool0")
    if ur5_chain.getNrOfJoints() == 0:
        rospy.logerr(f"Failed to get KDL Chain between 'base_link' and 'tool0'. Check link names in URDF.")
        return None
    
    rospy.loginfo(f"KDL Chain for UR5 created with {ur5_chain.getNrOfJoints()} joints.")
    return ur5_chain

# --- 2. Direct Dynamics Simulation Function (using Newton-Euler via PyKDL) ---
def simulate_direct_dynamics(ur5_chain, initial_q, initial_q_dot, applied_torques_func):
    num_joints = ur5_chain.getNrOfJoints()
    
    gravity_vector = PyKDL.Vector(0.0, 0.0, -9.81) 
    id_solver = PyKDL.ChainDynParam(ur5_chain, gravity_vector) 

    q = PyKDL.JntArray(num_joints)
    q_dot = PyKDL.JntArray(num_joints)
    
    for i in range(num_joints):
        q[i] = initial_q[i]
        q_dot[i] = initial_q_dot[i]
    
    time_points = []
    q_history = []
    q_dot_history = []
    q_ddot_history = [] 
    tau_applied_history = [] # <-- ADICIONADO: Histórico de torques aplicados
    
    current_time = 0.0
    
    rospy.loginfo("Iniciando simulação da dinâmica direta (Newton-Euler via PyKDL)...")
    while current_time <= SIM_DURATION:
        tau_applied_np = applied_torques_func(current_time)
        
        mass_matrix_kdl = PyKDL.JntSpaceInertiaMatrix(num_joints) 
        coriolis_centrifugal_terms_kdl = PyKDL.JntArray(num_joints)
        gravity_terms_kdl = PyKDL.JntArray(num_joints)

        id_solver.JntToMass(q, mass_matrix_kdl)       
        id_solver.JntToCoriolis(q, q_dot, coriolis_centrifugal_terms_kdl) 
        id_solver.JntToGravity(q, gravity_terms_kdl)   

        M_np = np.zeros((num_joints, num_joints))
        for i in range(num_joints):
            for j in range(num_joints):
                M_np[i, j] = mass_matrix_kdl[i, j] 
        
        C_np = np.array([coriolis_centrifugal_terms_kdl[i] for i in range(num_joints)])
        G_np = np.array([gravity_terms_kdl[i] for i in range(num_joints)])
        
        try:
            M_inv = np.linalg.inv(M_np)
            rhs = tau_applied_np - C_np - G_np
            q_ddot_np = M_inv @ rhs
        except np.linalg.LinAlgError:
            rospy.logwarn("Mass matrix is singular at this configuration. Stopping simulation.")
            break 

        for i in range(num_joints):
            q_dot[i] += q_ddot_np[i] * DT
            q[i] += q_dot[i] * DT
            
        time_points.append(current_time)
        q_history.append([q[i] for i in range(num_joints)])
        q_dot_history.append([q_dot[i] for i in range(num_joints)])
        q_ddot_history.append(q_ddot_np.tolist()) 
        tau_applied_history.append(tau_applied_np.tolist()) # <-- ADICIONADO: Armazenar os torques aplicados

        current_time += DT
    
    rospy.loginfo("Simulação da dinâmica direta (Newton-Euler via PyKDL) concluída.")
    return np.array(time_points), np.array(q_history), np.array(q_dot_history), np.array(q_ddot_history), np.array(tau_applied_history) # <-- RETORNA O HISTÓRICO DE TORQUES

# --- Main function to run the study ---
def main():
    rospy.init_node('ur5_dynamic_study', anonymous=True)

    ur5_chain = load_ur5_kdl_chain()
    if ur5_chain is None:
        return

    # initial_q = np.array([0.0, -np.pi/2, np.pi/2, 0.0, 0.0, 0.0]) 
    initial_q = np.zeros(6) 
    initial_q_dot = np.zeros(6) 

    torque_limits = np.array([100, 100, 100, 20, 20, 20]) 
    # random_torques = np.random.uniform(-torque_limits, torque_limits)
    random_torques = np.array([50.0, -80.0, 30.0, 1.0, -0.5, 0.8])
    rospy.loginfo(f"\nRandom torques applied for the first {0.5}s: {random_torques} Nm")

    def get_applied_torques(t):
        if t <= 0.5: 
            return random_torques
        else:
            return np.zeros(6) 

    # <-- MODIFICAÇÃO AQUI: Receber o histórico de torques
    time_points, q_history, q_dot_history, q_ddot_history, tau_applied_history = \
        simulate_direct_dynamics(ur5_chain, initial_q, initial_q_dot, get_applied_torques)

    # <-- MODIFICAÇÃO AQUI: Salvar o histórico de torques
    np.savez('kdl_sim_results.npz', times=time_points, q=q_history, q_dot=q_dot_history, q_ddot=q_ddot_history, tau_applied=tau_applied_history)
    rospy.loginfo("KDL simulation results (Newton-Euler via PyKDL) saved to 'kdl_sim_results.npz'.")

    # --- Plotting Results (Incluindo Torques) ---
    plt.figure(figsize=(15, 12)) # Ajustado para 3 linhas de plots
    joint_names = ['Shoulder Pan', 'Shoulder Lift', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3']

    for i in range(q_history.shape[1]):
        # Posição
        plt.subplot(6, 2, 2*i + 1) # 6 linhas, 2 colunas para Posição/Velocidade/Aceleração (coluna 1) e Torque (coluna 2)
        plt.plot(time_points, np.degrees(q_history[:, i]), label='Position (deg)', color='blue')
        plt.plot(time_points, np.degrees(q_dot_history[:, i]), label='Velocity (deg/s)', color='green', linestyle='--')
        plt.plot(time_points, np.degrees(q_ddot_history[:, i]), label='Acceleration (deg/s^2)', color='red', linestyle=':')
        plt.title(f'Junta {i+1} ({joint_names[i]}) - Movimento')
        plt.xlabel('Time (s)')
        plt.ylabel('Values')
        plt.grid(True)
        if i == 0: 
            plt.legend() 
        
        # Torque
        plt.subplot(6, 2, 2*i + 2) # Plot de torque na coluna 2
        plt.plot(time_points, tau_applied_history[:, i], label='Applied Torque (Nm)', color='purple')
        plt.title(f'Junta {i+1} ({joint_names[i]}) - Torque Aplicado')
        plt.xlabel('Time (s)')
        plt.ylabel('Torque (Nm)')
        plt.grid(True)
        if i == 0:
            plt.legend()

    plt.tight_layout()
    plt.suptitle('UR5 Direct Dynamics Simulation (Newton-Euler via PyKDL) with Torques', y=1.02)
    plt.show()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass