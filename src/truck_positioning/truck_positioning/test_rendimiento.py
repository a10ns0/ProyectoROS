import time
import requests
import matplotlib.pyplot as plt
import numpy as np


#COMPARACION DE RENDIMIENTO PERSISTENTE VS NO PERSISTENTE
# CONFIGURACIÓN
IP = "192.168.1.88"
PORT = "8000"
GRUA = "STS-006"
URL = f"http://{IP}:{PORT}/gruas/{GRUA}/trolleyPos"
NUM_PETICIONES = 50  # Cantidad de pruebas

print(f"--- INICIANDO BENCHMARK DE RED CONTRA {IP} ---")
print(f"Realizando {NUM_PETICIONES} peticiones por método...\n")

# --- MÉTODO 1: NO PERSISTENTE (Antiguo) ---
tiempos_no_persistente = []
print("1. Probando Conexión NO PERSISTENTE (Abriendo/Cerrando)...")

for i in range(NUM_PETICIONES):
    inicio = time.time()
    try:
        requests.get(URL, timeout=1) # Abre socket nuevo cada vez
        fin = time.time()
        tiempos_no_persistente.append((fin - inicio) * 1000) # Convertir a ms
    except Exception as e:
        print("x", end="", flush=True)

print(f"\n   -> Promedio: {np.mean(tiempos_no_persistente):.2f} ms")


# --- MÉTODO 2: PERSISTENTE (Nuevo) ---
tiempos_persistente = []
print("\n2. Probando Conexión PERSISTENTE (Session Keep-Alive)...")

session = requests.Session()
# Hacemos una primera petición "dummy" para pagar el coste del primer handshake
session.get(URL, timeout=1) 

for i in range(NUM_PETICIONES):
    inicio = time.time()
    try:
        session.get(URL, timeout=1) # Reusa el socket abierto
        fin = time.time()
        tiempos_persistente.append((fin - inicio) * 1000)
    except Exception as e:
        print("x", end="", flush=True)

session.close()
print(f"\n   -> Promedio: {np.mean(tiempos_persistente):.2f} ms")

# --- GENERACIÓN DEL GRÁFICO ---
print("\nGenerando gráfico comparativo...")

plt.figure(figsize=(10, 6))

# Gráfico de Líneas (Evolución)
plt.subplot(2, 1, 1)
plt.plot(tiempos_no_persistente, label='No Persistente (Standard)', color='red', marker='o', markersize=3, alpha=0.6)
plt.plot(tiempos_persistente, label='Persistente (Session)', color='green', marker='o', markersize=3)
plt.title('Latencia de Red: Conexión Normal vs Persistente')
plt.ylabel('Tiempo de Respuesta (ms)')
plt.legend()
plt.grid(True, linestyle='--', alpha=0.7)

# Gráfico de Barras (Promedios)
plt.subplot(2, 1, 2)
metodos = ['No Persistente', 'Persistente']
promedios = [np.mean(tiempos_no_persistente), np.mean(tiempos_persistente)]
colores = ['red', 'green']
barras = plt.bar(metodos, promedios, color=colores, alpha=0.8)

# Poner etiquetas de valor sobre las barras
for bar in barras:
    yval = bar.get_height()
    plt.text(bar.get_x() + bar.get_width()/2, yval + 0.5, f"{yval:.1f} ms", ha='center', va='bottom', fontweight='bold')

plt.ylabel('Tiempo Promedio (ms)')
plt.title('Comparación de Promedios')

plt.tight_layout()
plt.show() # Muestra la ventana con el gráfico