import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Parâmetros do APF
K_ATT = 1.0
K_REP = 100.0
RHO_0 = 2.5
STEP_SIZE = 0.1
obstacles = [[3.0, 6.0, 6.0], [5.0, 3.0, 3.0]] # Lista de obstáculos

# Objetivo
GOAL = np.array([10.0, 10.0, 10.0], dtype=float)

# Posição inicial
START_POSITION = np.array([0.0, 0.0, 0.0], dtype=float)


def set_obstacles(obstacles_list):
    """Define a lista de obstáculos."""
    global obstacles
    obstacles = [np.array(obstacle, dtype=float) for obstacle in obstacles_list]


def attractive_force(position, goal):
    """Calcula a força atrativa em 3D."""
    return -K_ATT * (np.array(position) - np.array(goal))


def repulsive_force(position, obstacle):
    """Calcula a força repulsiva em 3D."""
    position = np.array(position)
    obstacle = np.array(obstacle)
    dist = np.linalg.norm(position - obstacle)
    if dist <= RHO_0:
        force_magnitude = K_REP * (1/dist - 1/RHO_0) / dist**2
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
    return np.array(position) + STEP_SIZE * np.array(force)


def simulate(start_position, goal, obstacles_list, max_iterations=1000, tolerance=0.2):
    """Simula o movimento até o objetivo ou o número máximo de iterações."""
    global obstacles
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


def plot_results(path, goal, obstacles, start_position):
    """Plota o caminho, obstáculos e objetivo em 3D."""
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plotando o caminho
    ax.plot(path[:, 0], path[:, 1], path[:, 2], label="Path", linewidth=2, color="blue")

    # Plotando os obstáculos
    for obstacle in obstacles:
        ax.scatter(obstacle[0], obstacle[1], obstacle[2], color='red', marker='o', label="Obstacle", s=200)

    # Plotando o objetivo
    ax.scatter(goal[0], goal[1], goal[2], color='green', marker='x', label="Goal", s=200)

    # Plotando a posição inicial
    ax.scatter(start_position[0], start_position[1], start_position[2], color='black', marker='s', label="Start", s=100)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Artificial Potential Field 3D (Procedural)")
    ax.legend()
    plt.show()


if __name__ == "__main__":
    # print(simulate(START_POSITION, GOAL, obstacles))
    path = simulate(START_POSITION, GOAL, obstacles)
    plot_results(path, GOAL, obstacles, START_POSITION)

    

   