import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class PotentialField:
    def __init__(self, k_att, k_rep, rho_0, step_size):
        self.k_att = k_att
        self.k_rep = k_rep
        self.rho_0 = rho_0
        self.step_size = step_size
        self.obstacles = [[3.0, 6.0, 6.0], [5.0, 3.0, 3.0]]  # Lista de obstáculos

    def set_obstacles(self, obstacles):
        """Define a lista de obstáculos."""
        self.obstacles = [np.array(obstacle, dtype=float) for obstacle in obstacles]
        return self.obstacles

    def attractive_potential(self, position, goal):
        """Calcula o potencial atrativo em 3D."""
        return 0.5 * self.k_att * np.linalg.norm(np.array(position) - np.array(goal))**2

    def repulsive_potential(self, position, obstacle):
        """Calcula o potencial repulsivo em 3D."""
        position = np.array(position)
        obstacle = np.array(obstacle)
        dist = np.linalg.norm(position - obstacle)
        if dist <= self.rho_0:
            return 0.5 * self.k_rep * (1/dist - 1/self.rho_0)**2
        return 0.0

    def attractive_force(self, position, goal):
        """Calcula a força atrativa em 3D."""
        return -self.k_att * (np.array(position) - np.array(goal))

    def repulsive_force(self, position, obstacle):
        """Calcula a força repulsiva em 3D."""
        position = np.array(position)
        obstacle = np.array(obstacle)
        dist = np.linalg.norm(position - obstacle)
        if dist <= self.rho_0:
            force_magnitude = self.k_rep * (1/dist - 1/self.rho_0) / dist**2
            force_direction = position - obstacle
            return force_magnitude * force_direction
        return np.array([0.0, 0.0, 0.0])

    def total_force(self, position, goal):
        """Calcula a força total em 3D, considerando múltiplos obstáculos."""
        total_repulsive_force = np.array([0.0, 0.0, 0.0])
        for obstacle in self.obstacles:
            total_repulsive_force += self.repulsive_force(position, obstacle)
        return self.attractive_force(position, goal) + total_repulsive_force

    def update_position(self, position, force):
        """Atualiza a posição em 3D com base na força e no passo de aprendizado."""
        return np.array(position) + self.step_size * np.array(force)

class Simulator:
    def __init__(self, potential_field, goal, obstacles, start_position):
        self.potential_field = potential_field
        self.goal = np.array(goal)
        self.obstacles = [np.array(obstacle) for obstacle in obstacles]
        self.current_position = np.array(start_position, dtype=float)
        self.path = [self.current_position]

    def simulate(self, max_iterations=1000, tolerance=0.2):
        """Simula o movimento até o objetivo ou o número máximo de iterações."""
        iterations = 0
        while np.linalg.norm(self.current_position - self.goal) > tolerance and iterations < max_iterations:
          force = self.potential_field.total_force(self.current_position, self.goal)
          self.current_position = self.potential_field.update_position(self.current_position, force)
          self.path.append(self.current_position)
          iterations += 1
        return np.array(self.path)

class Plotter:
    def __init__(self, path, goal, obstacles, start_position):
        self.path = path
        self.goal = goal
        self.obstacles = obstacles
        self.start_position = start_position

    def plot(self):
        """Plota o caminho, obstáculos e objetivo em 3D."""
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Plotando o caminho
        ax.plot(self.path[:, 0], self.path[:, 1], self.path[:, 2], label="Path", linewidth=2, color="blue")

        # Plotando os obstáculos
        for obstacle in self.obstacles:
            ax.scatter(obstacle[0], obstacle[1], obstacle[2], color='red', marker='o', label="Obstacle", s=200)

        # Plotando o objetivo
        ax.scatter(self.goal[0], self.goal[1], self.goal[2], color='green', marker='x', label="Goal", s=200)

        # Plotando a posição inicial
        ax.scatter(self.start_position[0], self.start_position[1], self.start_position[2], color='black', marker='s', label="Start", s=100)

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("Artificial Potential Field 3D (OOP)")
        ax.legend()
        plt.show()

if __name__ == "__main__":
    # Parâmetros
    K_ATT = 1.0
    K_REP = 100.0
    RHO_0 = 200
    STEP_SIZE = 0.1

    # Instâncias
    potential_field = PotentialField(K_ATT, K_REP, RHO_0, STEP_SIZE)
    start_position = [0.0, 0.0, 0.0]
    goal = [10.0, 10.0, 10.0]
    obstacles = [[3.0, 6.0, 6.0], [5.0, 3.0, 3.0]] # Mais obstáculos
   
    simulator = Simulator(potential_field, goal, obstacles, start_position)
    path = simulator.simulate()

    plotter = Plotter(path, goal, obstacles, start_position)
    plotter.plot()

    
   