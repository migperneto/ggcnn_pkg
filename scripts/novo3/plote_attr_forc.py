import pandas as pd
import matplotlib.pyplot as plt

# Carregar dados do arquivo CSV 
df = pd.read_csv('scripts/attr_force_hist.csv')

# df.columns = ['x', 'y', 'z']

# Converter colunas para tipo numérico
df['x'] = pd.to_numeric(df['x'], errors='coerce')
df['y'] = pd.to_numeric(df['y'], errors='coerce')
df['z'] = pd.to_numeric(df['z'], errors='coerce')

# Verificar se as colunas necessárias existem
if {'x', 'y', 'z'}.issubset(df.columns):
    # Criar a figura
    plt.figure(figsize=(10, 6))
    
    # Plotar todos os eixos no mesmo gráfico
    plt.plot(df.index.to_numpy(), df['x'].to_numpy(), color='r', label='Eixo x')
    plt.plot(df.index.to_numpy(), df['y'].to_numpy(), color='g', label='Eixo y')
    plt.plot(df.index.to_numpy(), df['z'].to_numpy(), color='b', label='Eixo z')
    
    # Configurações do gráfico
    plt.xlabel('Índice')
    plt.ylabel('Valores')
    plt.title('Valore da força de atração Fx, Fy e Fz ao longo do tempo')
    plt.legend()
    plt.grid()
    
    # Exibir o gráfico
    plt.show()
else:
    print("O arquivo CSV não contém as colunas 'x', 'y' e 'z'.")
