import numpy as np
import matplotlib.pyplot as plt

class PotentialField:
    def __init__(self, k_att, k_rep, rho_0, step_size):
        self.k_att = k_att
        self.k_rep = k_rep
        self.rho_0 = rho_0
        self.step_size = step_size

    def attractive_potential(self, position, goal):
      """Calcula o potencial atrativo."""
      return 0.5 * self.k_att * np.linalg.norm(position - goal)**2

    def repulsive_potential(self, position, obstacle):
      """Calcula o potencial repulsivo."""
      dist = np.linalg.norm(position - obstacle)
      if dist <= self.rho_0:
        return 0.5 * self.k_rep * (1/dist - 1/self.rho_0)**2
      return 0.0

    def attractive_force(self, position, goal):
      """Calcula a força atrativa."""
      return -self.k_att * (position - goal)

    def repulsive_force(self, position, obstacle):
      """Calcula a força repulsiva."""
      dist = np.linalg.norm(position - obstacle)
      if dist <= self.rho_0:
        force_magnitude = self.k_rep * (1/dist - 1/self.rho_0) / dist**2
        force_direction = position - obstacle
        return force_magnitude * force_direction
      return np.array([0.0, 0.0])

    def total_force(self, position, goal, obstacles):
        """Calcula a força total, considerando múltiplos obstáculos."""
        total_repulsive_force = np.array([0.0, 0.0])
        for obstacle in obstacles:
            total_repulsive_force += self.repulsive_force(position, obstacle)
        return self.attractive_force(position, goal) + total_repulsive_force

    def update_position(self, position, force):
        """Atualiza a posição com base na força e no passo de aprendizado."""
        return position + self.step_size * force

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
          force = self.potential_field.total_force(self.current_position, self.goal, self.obstacles)
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
        """Plota o caminho, obstáculos e objetivo."""
        plt.figure(figsize=(8, 8))
        plt.plot(self.path[:,0], self.path[:,1], label="Path", linewidth=2, color="blue")
        for obstacle in self.obstacles:
            plt.scatter(*obstacle, color='red', marker='o', label="Obstacle", s=200)
        plt.scatter(*self.goal, color='green', marker='x', label="Goal", s=200)
        plt.scatter(*self.start_position, color='black', marker='s', label="Start", s=100)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Artificial Potential Field 2D (OOP)")
        plt.grid(True)
        plt.legend()
        plt.show()


if __name__ == "__main__":
    # Parâmetros
    K_ATT = 1.0
    K_REP = 100.0
    RHO_0 = 2.5
    STEP_SIZE = 0.1

    # Instâncias
    potential_field = PotentialField(K_ATT, K_REP, RHO_0, STEP_SIZE)
    start_position = [0.0, 0.0]
    goal = [10.0, 10.0]
    obstacles = [[5.0, 6.0], [8.0, 4.0]] # Mais obstáculos

    simulator = Simulator(potential_field, goal, obstacles, start_position)
    path = simulator.simulate()

    plotter = Plotter(path, goal, obstacles, start_position)
    plotter.plot()