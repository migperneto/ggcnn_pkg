#!/usr/bin/env python3
"""
Neste arquivo a força de atração é calculada utilizando a combinação do potencial 
canônico com o potencial parabólico. (porém esse método apresentou problemas na equaçao canônica)
Foi implementado força de repulsão com melhoramento do Campo Potencial Artificial Adaptativo
será utilizado matrix jacobiana para calcular as forças de atração e repulsão
"""

import numpy as np

class PotentialField:
    def __init__(self, k_att_val, k_rep_val, rho_0_val, step_size_val, d):
        """Define os parâmetros do campo potencial."""
        self.k_att = k_att_val
        self.k_rep = k_rep_val
        self.rho_0 = rho_0_val
        self.step_size = step_size_val
        self.obstacles = [] # Lista de obstáculos
        self.rho = 0
        self.iterations = 0
        self.attr_force_hist = [] #Lista contendo as forças de atração
        self.rep_force_hist = [] #Lista contendo as forças de repulsão
        self.tot_forc_hist = [] #Lista contendo a força resultante da atração e repulsão
        self.d = d

    def set_obstacles(self, obstacles_list):
        """Define a lista de obstáculos."""
        self.obstacles = [np.array(obstacle, dtype=float) for obstacle in obstacles_list]
        # return self.obstacles
    
    # Calculo da força de atração utilizando a combinação do potencial conico e parabólico
    def attractive_force(self, position, goal):
        """Calcula a força atrativa em 3D."""
        d = self.d
        if abs(np.linalg.norm(np.array(position,dtype=float) - np.array(goal,dtype=float))) <= d: # pequenas distâncias utiliza-se o potencial parabólico
            return -self.k_att * (np.array(position) - np.array(goal))
        elif abs(np.linalg.norm(np.array(position,dtype=float) - np.array(goal,dtype=float))) > d: # grande distância utiliza-se o potencial canônico
            return -d*self.k_att * ((np.array(position) - np.array(goal)) / abs(np.array(position) - np.array(goal)))

    # # Calculo da força de atração somente utilizando o potencial parabólico
    # def attractive_force(self, position, goal):
    #     """Calcula a força atrativa em 3D."""
    #     attrac_force = -self.k_att * (np.array(position) - np.array(goal))
    #     return attrac_force

        
    #cálculo da força de repulsão com melhoramento do Campo Potencial Artificial Adaptativo
    def repulsive_force(self, position, obstacle, goal):
        """Calcula a força repulsiva em 3D."""
        position = np.array(position)
        obstacle = np.array(obstacle)
        dist = np.linalg.norm(position - obstacle)
        ro_g = np.linalg.norm(position - goal)
        n = 3
        if dist <= self.rho_0:
            #força 1
            force_magnitude_1 = self.k_rep * (1/dist - 1/self.rho_0)*(ro_g**n/(dist**2*(1+ro_g**n)))
            force_direction_or = position - obstacle
            #força 2
            force_magnitude_2 = (n/2)*self.k_rep * ((1/dist - 1/self.rho_0)**2)*(ro_g**(n-1)/(1+ro_g**n))
            force_direction_rg = position - goal

            return ((force_magnitude_1 * force_direction_or) + (force_magnitude_2 * force_direction_rg))
        return np.array([0.0, 0.0, 0.0], dtype=float)

    def total_force(self, position, goal):
        """Calcula a força total em 3D, considerando múltiplos obstáculos."""
        total_repulsive_force = np.array([0.0, 0.0, 0.0], dtype=float)
        for obstacle in self.obstacles:
            total_repulsive_force += self.repulsive_force(position, obstacle, goal)
        att_force = self.attractive_force(position, goal)
        tot_forc = att_force + total_repulsive_force 
        self.tot_forc_hist.append(tot_forc)
        self.attr_force_hist.append(att_force) 
        self.rep_force_hist.append(total_repulsive_force)
        return tot_forc, self.attr_force_hist, self.rep_force_hist


    def update_position(self, position, force):
        """Atualiza a posição em 3D com base na força e no passo de aprendizado."""
        up_pos = np.array(position) + self.step_size * np.array(force)
        return up_pos

    def simulate(self, start_position, goal, obstacles_list, max_iterations=100, tolerance=0.1):
        """Simula o movimento até o objetivo ou o número máximo de iterações."""

        self.set_obstacles(obstacles_list)
        current_position = np.array(start_position, dtype=float)
        path = [current_position]
        self.iterations = 0

        # self.rho = np.linalg.norm(current_position - goal)
        while np.linalg.norm(current_position - goal) > tolerance and self.iterations < max_iterations:
            force = self.total_force(current_position, goal)
            current_position = self.update_position(current_position, force[0])
            path.append(current_position)
            self.iterations += 1
        
        tril = np.array(path)
        return tril
    
    def forces_hist(self):
        return self.tot_forc_hist, self.attr_force_hist, self.rep_force_hist
    
    







# cp = PotentialField(k_att_val = 1.0, k_rep_val = 0.1, rho_0_val = 0.1, step_size_val = 0.01)
# print('força de atração')
# print(cp.attractive_force([-0.0953, -0.1914, 0.9063], [-0.0866, -0.2728, 0.9036]))
# print('força de repulsão')
# print(cp.repulsive_force([-0.0953, -0.1914, 0.9063], [0, 0, 0]))
# print('força total')
# print(cp.total_force([-0.0953, -0.1914, 0.9063], [-0.0866, -0.2728, 0.9036]))


# obstacles_list = [[-0.2, -0.1915, 0.72], [-0.3, -0.191, 0.65]]
# cp.set_obstacles(obstacles_list)   #chamando o método

# print(cp.obstacles)  #imprimindo a lista de obstáculos
# print(cp.step_size)  #imprimindo o passo

# path = cp.simulate([-0.0953, -0.1914, 0.9063], [-0.3970, -0.1914, 0.5858], [])

# print(path)

