#!/src/bin/ven python3

import numpy as np

# --- Initial Parameters for UR5 Dynamic Study ---

def get_initial_params():
    """
    Returns the initial parameters for the UR5 dynamic study.
    
    Returns:
        dict: A dictionary containing the initial joint positions, velocities, and torques.
    """
    torque_limits = np.array([100, 100.0, 100.0, 15.0, 15.0, 15.0], dtype=np.float64) 
    random_torques = np.random.uniform(-torque_limits, torque_limits)

    initial_params = {
        # 'q_initial': np.array([0.0, -np.pi/2, np.pi/2, 0.0, 0.0, 0.0]),  # Initial joint positions (rad)
        'q_initial': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        'dq_initial': np.zeros(6),  # Initial joint velocities (rad/s)
        # 'dq_initial': [0.1, 0.05, -0.02, 0.0, 0.0, 0.0],
        # 'tau_apply': random_torques   # joint torques (Nm)
        'tau_apply': [-100, 0, 0, 0, 0, 0]
    }
    
    return initial_params


# params = get_initial_params()

# print(params['q_initial'])

# Torques para simulação:
''' 
[10.0, 20.0, 5.0, 1.0, 0.5, 0.1]
 '''


'''
Força nas juntas 
[-100, 0, 0, 0, 0, 0]
[100, 0, 0.0, 0, 0, 0]
[0, 0, -30, 0, 0, 0]
[0, 0, 30, 0, 0, 0]
[0, 0, 0, 0, -10, 0]
[0, 0, 0, 0, 10, 0]
[0, 0, 0, 0, 0, -10]
[0, 0, 0, 0, 0, 10]
[0, 0, 0, 10, 0, 0]
[10.0, 20.0, 5.0, 1.0, 0.5, 0.1]
[100.0, 100.0, 100.0, 15.0, 15.0, 15.0]
'''