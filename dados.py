import pandas as pd
import matplotlib.pyplot as plt

# Lê o arquivo CSV
df = pd.read_csv("dados29072025130026.csv")

# Cria coluna unificada com Data + Hora
df['tempo'] = pd.to_datetime(df['Data'] + ' ' + df['Hora'], format='%d/%m/%y %H:%M:%S')

# Define 'tempo' como índice
df.set_index('tempo', inplace=True)

# --- Plot Aceleração ---
plt.figure(figsize=(10, 5))
plt.plot(df.index, df['AccX'], label='AccX')
plt.plot(df.index, df['AccY'], label='AccY')
plt.plot(df.index, df['AccZ'], label='AccZ')
plt.title('Aceleração - MPU6050')
plt.xlabel('Tempo')
plt.ylabel('Aceleração (raw)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Plot Giroscópio ---
plt.figure(figsize=(10, 5))
plt.plot(df.index, df['GyroX'], label='GyroX')
plt.plot(df.index, df['GyroY'], label='GyroY')
plt.plot(df.index, df['GyroZ'], label='GyroZ')
plt.title('Giroscópio - MPU6050')
plt.xlabel('Tempo')
plt.ylabel('Velocidade Angular (raw)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Plot Temperatura ---
plt.figure(figsize=(10, 4))
plt.plot(df.index, df['Temperatura'], color='orange')
plt.title('Temperatura - MPU6050')
plt.xlabel('Tempo')
plt.ylabel('Temperatura (°C)')
plt.grid(True)
plt.tight_layout()
plt.show()
