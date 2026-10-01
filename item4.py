import time
from pymycobot.mycobot import MyCobot

mc = MyCobot("/dev/ttyUSB0", 1000000)
g_speed = 40

# Objetivo cartesiano: [x, y, z, rx, ry, rz]  (mm y grados)
objetivo = [125, -99, 250, -90, -45, -90]

# mode=0 -> movimiento angular (el firmware resuelve la IK y elige la solución)
# mode=1 -> movimiento lineal
mc.send_coords(objetivo, g_speed, 0)

# Esperar a que el brazo termine de moverse
time.sleep(0.5)  # margen para que el firmware arranque el movimiento
while mc.is_moving() == 1:
    time.sleep(0.1)
time.sleep(0.5)  # margen para que se asiente

# Leer los ángulos realmente ejecutados
angulos = mc.get_angles()
print("Ángulos ejecutados (grados):", angulos)