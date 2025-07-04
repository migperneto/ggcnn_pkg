#!/usr/bin/env python3

import numpy as np
from ur5_dynamic_equations import ur5_matrices

# --- Inverse Dynamics Function ---
def inverse_dynamics_ur5(q, dq, q_ddot):
    """
    Calculates the joint torques (tau) required to achieve a desired
    joint acceleration (q_ddot), given joint positions (q) and velocities (dq).
    
    tau = M * q_ddot + CDq + G (ignoring friction)

    Args:
        q (np.array): 6x1 array of joint positions (rad).
        dq (np.array): 6x1 array of joint velocities (rad/s).
        q_ddot (np.array): 6x1 array of desired joint accelerations (rad/s^2).

    Returns:
        np.array: 6x1 array of joint torques (Nm).
    """
    M, G, CDq = ur5_matrices(q, dq)
    
    # Ensure q_ddot is a column vector for matrix multiplication
    q_ddot_col = np.array(q_ddot).reshape(-1, 1)
    
    # tau = M * q_ddot + CDq + G
    tau = M @ q_ddot_col + CDq + G
    
    return tau