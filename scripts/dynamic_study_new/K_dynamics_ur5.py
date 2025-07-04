#!/usr/bin/env python3

import numpy as np
from ur5_dynamic_equations import ur5_matrices

# --- Direct Dynamics Function ---
def direct_dynamics_ur5(q, dq, tau):
    """
    Calculates the resulting joint accelerations (q_ddot) given
    joint positions (q), velocities (dq), and applied joint torques (tau).

    q_ddot = M_inv * (tau - CDq - G) (ignoring friction)

    Args:
        q (np.array): 6x1 array of joint positions (rad).
        dq (np.array): 6x1 array of joint velocities (rad/s).
        tau (np.array): 6x1 array of applied joint torques (Nm).

    Returns:
        np.array: 6x1 array of resulting joint accelerations (rad/s^2).
    """
    M, G, CDq = ur5_matrices(q, dq)
    
    # Ensure tau is a column vector
    tau_col = np.array(tau).reshape(-1, 1)
    
    # q_ddot = M_inv * (tau - CDq - G)
    try:
        M_inv = np.linalg.inv(M)
        q_ddot = M_inv @ (tau_col - CDq - G)
    except np.linalg.LinAlgError:
        print("Error: Mass matrix is singular, cannot perform direct dynamics.")
        q_ddot = np.full((6, 1), np.nan) # Return NaN array if matrix is singular
        
    return q_ddot