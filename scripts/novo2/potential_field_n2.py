import numpy as np

class PotentialField:
    def __init__(self, k_att_val, k_rep_val, rho_0_val, step_size_val):
        """Define os parâmetros do campo potencial."""
        self.k_att = k_att_val
        self.k_rep = k_rep_val
        self.rho_0 = rho_0_val
        self.step_size = step_size_val
        self.obstacles = [] # Lista de obstáculos

    def set_obstacles(self, obstacles_list):
        """Define a lista de obstáculos."""
        self.obstacles = [np.array(obstacle, dtype=float) for obstacle in obstacles_list]

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
        return np.array([0.0, 0.0, 0.0], dtype=float)

    def total_force(self, position, goal):
        """Calcula a força total em 3D, considerando múltiplos obstáculos."""
        total_repulsive_force = np.array([0.0, 0.0, 0.0], dtype=float)
        for obstacle in self.obstacles:
            total_repulsive_force += self.repulsive_force(position, obstacle)
        return self.attractive_force(position, goal) + total_repulsive_force

    def update_position(self, position, force):
        """Atualiza a posição em 3D com base na força e no passo de aprendizado."""
        return np.array(position) + self.step_size * np.array(force)

    def simulate(self, start_position, goal, obstacles_list, max_iterations=1000, tolerance=0.2):
        """Simula o movimento até o objetivo ou o número máximo de iterações."""
        self.set_obstacles(obstacles_list)
        current_position = np.array(start_position, dtype=float)
        path = [current_position]
        iterations = 0

        while np.linalg.norm(current_position - goal) > tolerance and iterations < max_iterations:
            force = self.total_force(current_position, goal)
            current_position = self.update_position(current_position, force)
            path.append(current_position)
            iterations += 1
        return np.array(path)
    
    