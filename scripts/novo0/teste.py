import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# plotando um gráfico de uma senoide em 3D
# figura = plt.figure()
# grafico = figura.gca(projection='3d')
# z = np.linspace(-3, 3, 200)
# y = np.linspace(-10, 10, 200)
# x = z * np.sin(y)

# grafico.plot(x,y,z, color = 'red')

# plt.show()
 


# ********************************************
# Plotando uma esfera em 3D  
# Definindo os intervalos de theta e phi

# theta = np.arange(0, 2*np.pi, 0.01)
# phi = np.arange(0, 2*np.pi, 0.01)

# # Definindo o raio da esfera
# r = 1 

# # criando as coordenadas para 3D
# theta, phi = np.meshgrid(theta, phi)

# # Eqs no sistema de coodenadas esféricas
# x = r * np.sin(phi) * np.cos(theta)
# y = r * np.sin(phi * np.sin(theta))
# z = r * np.cos(phi)

# # Vinculando a figura às coordenadas 3D
# fig = plt.figure()
# ax = Axes3D(fig)

# # Plotando o gráfico da esfera
# ax.plot_surface(x,y,z)
# plt.show()


# Fazer um cadastro de viagem (Deve pedir o nome do viajante, dar as opções de destino e imprimir a selecionada)


# class calc_media:
#     def __init__(self):
#         self.alunos = []

#     def add_alunos(self):
#         for i in range(3):
#             self.nome = input('Informe o nome do aluno: ')
#             self.alunos.append(self.nome)       
#         print(self.alunos)

# turma1 = calc_media()
# turma1.add_alunos()


class biblioteca:
    def __init__(self, livros):
        self.livros = livros

    def solicitacoes(self):
        self.nome = input('Informe o seu nome: ')
        print(self.livros)
        self.escolha = int(input('Escolha uma opção: '))
        if self.escolha > 3:
            while(self.escolha > 3):
                self.escolha = int(input(('Por favor, digiter um número menor ou igual a três: ')))
        else:
            print(f'O livro escolhido foi {self.escolha}')

    def imprimir_escolha(self):
        print(f'O livro escolhido foi {self.escolha}')


livros = ['[1] Livro 1', '[2] Livro 2', '[3] Livro 3']
biblioteca = biblioteca(livros)
biblioteca.solicitacoes()
biblioteca.imprimir_escolha()


