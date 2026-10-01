import math

def dh_matrix(d, a, alpha, theta):
    """Calcula la matriz de transformación homogénea 4x4 de un eslabón."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0]
    ]

def matmult(A, B):
    """Multiplicación de matrices 4x4."""
    C = [[0.0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            C[i][j] = sum(A[i][k] * B[k][j] for k in range(4))
    return C

def fk(q_deg):
    """
    Cinemática Directa basada en la Tabla DH del cuaderno.
    Entrada: q_deg = [q1, q2, q3, q4, q5, q6] en grados.
    Salida: [x, y, z] en milímetros.
    """
    # Convertir ángulos de entrada a radianes
    q = [math.radians(angle) for angle in q_deg]
    
    # Parámetros físicos (tomados de su tabla manuscrita)
    d1, d4, d5, d6 = 131.22, 63.4, 75.05, 45.6
    a2, a3 = -110.4, -96.0
    
    # Tabla DH: [d, a, alpha_rad, theta_rad]
    dh_table = [
        [d1,   0.0,  math.radians(90),  q[0]],
        [0.0,   a2,  math.radians(0),   q[1] - math.radians(90)],
        [0.0,   a3,  math.radians(0),   q[2]],
        [d4,   0.0,  math.radians(90),  q[3] - math.radians(90)],
        [d5,   0.0, -math.radians(90),  q[4] + math.radians(90)],
        [d6,   0.0,  math.radians(0),   q[5]]
    ]
    
    # Matriz Identidad Inicial (T0_0)
    T = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    
    # Multiplicar consecutivamente T = A1 * A2 * A3 * A4 * A5 * A6
    for row in dh_table:
        d, a, alpha, theta = row
        A_i = dh_matrix(d, a, alpha, theta)
        T = matmult(T, A_i)
        
    # Extraer la columna de posición (X, Y, Z)
    x = round(T[0][3], 2)
    y = round(T[1][3], 2)
    z = round(T[2][3], 2)
    
    return [x, y, z]

# ==============================================================================
# VALIDACIÓN Y PREDICCIÓN DE LAS 3 POSES REQUERIDAS
# ==============================================================================

# Pose 1: Posición Inicial (Home / Reposo)
pose1_q = [0, 0, 0, 0, 0, 45]
pred_pose1 = fk(pose1_q)

# Pose 2: Giro de Base a 90°
pose2_q = [90, 0, 0, 0, 0, 0]
pred_pose2 = fk(pose2_q)

# Pose 3: Flexión de Brazo (Ejemplo: Articulación 2 a 45° y 3 a -45°)
pose3_q = [0, 45, -45, 0, 0, 0]
pred_pose3 = fk(pose3_q)

print("--------------------------------------------------")
print(" PREDICCIONES DE CINEMÁTICA DIRECTA fk(q) ")
print("--------------------------------------------------")
print(f"Pose 1 {pose1_q} -> Predicción (x,y,z): {pred_pose1} mm")
print(f"Pose 2 {pose2_q} -> Predicción (x,y,z): {pred_pose2} mm")
print(f"Pose 3 {pose3_q} -> Predicción (x,y,z): {pred_pose3} mm")
print("--------------------------------------------------")
