import numpy as np

# Parâmetros do APF (serão definidos no nó principal)
k_att = None
k_rep = None
rho_0 = None
step_size = None
obstacles = [] # Lista de obstáculos

def set_potential_field_params(k_att_val, k_rep_val, rho_0_val, step_size_val):
    """Define os parâmetros do campo potencial."""
    global k_att, k_rep, rho_0, step_size
    k_att = k_att_val
    k_rep = k_rep_val
    rho_0 = rho_0_val
    step_size = step_size_val


def set_obstacles(obstacles_list):
    """Define a lista de obstáculos."""
    global obstacles
    obstacles = [np.array(obstacle, dtype=float) for obstacle in obstacles_list]


def attractive_potential(position, goal):
    """Calcula o potencial atrativo em 3D."""
    global k_att
    return 0.5 * k_att * np.linalg.norm(np.array(position) - np.array(goal))**2


def repulsive_potential(position, obstacle):
    """Calcula o potencial repulsivo em 3D."""
    global k_rep, rho_0
    position = np.array(position)
    obstacle = np.array(obstacle)
    dist = np.linalg.norm(position - obstacle)
    if dist <= rho_0:
        return 0.5 * k_rep * (1/dist - 1/rho_0)**2
    return 0.0


def attractive_force(position, goal):
    """Calcula a força atrativa em 3D."""
    global k_att
    return -k_att * (np.array(position) - np.array(goal))


def repulsive_force(position, obstacle):
    """Calcula a força repulsiva em 3D."""
    global k_rep, rho_0
    position = np.array(position)
    obstacle = np.array(obstacle)
    dist = np.linalg.norm(position - obstacle)
    if dist <= rho_0:
        force_magnitude = k_rep * (1/dist - 1/rho_0) / dist**2
        force_direction = position - obstacle
        return force_magnitude * force_direction
    return np.array([0.0, 0.0, 0.0], dtype=float)


def total_force(position, goal):
    """Calcula a força total em 3D, considerando múltiplos obstáculos."""
    global obstacles
    total_repulsive_force = np.array([0.0, 0.0, 0.0], dtype=float)
    for obstacle in obstacles:
        total_repulsive_force += repulsive_force(position, obstacle)
    return attractive_force(position, goal) + total_repulsive_force


def update_position(position, force):
    """Atualiza a posição em 3D com base na força e no passo de aprendizado."""
    global step_size
    return np.array(position) + step_size * np.array(force)


def simulate(start_position, goal, obstacles_list, max_iterations=1000, tolerance=0.2):
    """Simula o movimento até o objetivo ou o número máximo de iterações."""
    set_obstacles(obstacles_list)
    current_position = np.array(start_position, dtype=float)
    path = [current_position]
    iterations = 0

    while np.linalg.norm(current_position - goal) > tolerance and iterations < max_iterations:
        force = total_force(current_position, goal)
        current_position = update_position(current_position, force)
        path.append(current_position)
        iterations += 1
    return np.array(path)