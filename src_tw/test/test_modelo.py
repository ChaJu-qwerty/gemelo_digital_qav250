import sys
import os
import numpy as np

# Agregar el directorio src_tw al path para poder importar el módulo localmente
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from gemelo_digital_qav250.modelo_euler_lagrange import DroneModel

def run_tests():
    # Parámetros de prueba similares a qav250_params.yaml
    params = {
        "m": 0.650,       # kg
        "l": 0.125,       # m
        "Ixx": 0.0058,    # kg·m²
        "Iyy": 0.0058,    # kg·m²
        "Izz": 0.0100,    # kg·m²
        "k": 3.13e-05,    # N·s²/rad²
        "b": 7.50e-07,    # N·m·s²/rad²
        "Ir": 6.00e-05,   # kg·m²
        "Ax": 0.0,
        "Ay": 0.0,
        "Az": 0.0
    }

    print("=== INICIANDO PRUEBAS UNITARIAS DE DRONEMODEL ===")
    model = DroneModel(params)

    # 1. PRUEBA DE EMPUJE EN ESTACIONARIO (HOVER)
    # T = m * g = 0.650 * 9.81 = 6.3765 N
    # Cada motor debe empujar f_i = T/4 = 1.5941 N
    # f_i = k * omega_i^2 => omega_i = sqrt(f_i / k)
    f_i_hover = (params["m"] * 9.81) / 4.0
    omega_hover = np.sqrt(f_i_hover / params["k"])
    print(f"\n[HOVER] Velocidad calculada para hover: {omega_hover:.2f} rad/s")

    T, tau_phi, tau_theta, tau_psi, omega_G = model.calcular_fuerzas_torques(
        omega_hover, omega_hover, omega_hover, omega_hover
    )
    print(f"[HOVER] Empuje total: {T:.4f} N (Deseado: {params['m'] * 9.81:.4f} N)")
    print(f"[HOVER] Torques: Roll={tau_phi:.4f}, Pitch={tau_theta:.4f}, Yaw={tau_psi:.4f}")
    
    assert np.isclose(T, params["m"] * 9.81, atol=1e-3), "El empuje de hover no coincide con el peso"
    assert np.isclose(tau_phi, 0.0), "El torque de roll debería ser cero"
    assert np.isclose(tau_theta, 0.0), "El torque de pitch debería ser cero"
    assert np.isclose(tau_psi, 0.0), "El torque de yaw debería ser cero"
    print("[HOVER] OK!")

    # 2. PRUEBA DE MOVIMIENTO EN ROLL (INCLINACIÓN A LA IZQUIERDA/DERECHA)
    # Roll positivo (inclinación a la izquierda): Izquierda aumenta (2, 3), Derecha disminuye (1, 4)
    # Aumentamos omega en los motores 2 y 3, y disminuimos en 1 y 4
    omega_FR = omega_hover * 0.9  # Motor 1 (der)
    omega_RL = omega_hover * 1.1  # Motor 2 (izq)
    omega_FL = omega_hover * 1.1  # Motor 3 (izq)
    omega_RR = omega_hover * 0.9  # Motor 4 (der)

    _, tau_phi_roll, _, _, _ = model.calcular_fuerzas_torques(
        omega_FR, omega_RL, omega_FL, omega_RR
    )
    print(f"\n[ROLL] Aplicando comando de roll.")
    print(f"[ROLL] Torque de roll: {tau_phi_roll:.4f} (Debería ser > 0)")
    assert tau_phi_roll > 0.0, "El torque de roll debería ser positivo para inclinar a la izquierda"
    print("[ROLL] OK!")

    # 3. PRUEBA DE MOVIMIENTO EN PITCH (INCLINACIÓN HACIA ADELANTE/ATRÁS)
    # En Luukkonen, el empuje trasero aumenta el torque de pitch positivo (inclinando el dron hacia adelante)
    # Trasero aumenta (2, 4), Frontal disminuye (1, 3)
    omega_FR = omega_hover * 0.9  # Motor 1 (front)
    omega_RL = omega_hover * 1.1  # Motor 2 (rear)
    omega_FL = omega_hover * 0.9  # Motor 3 (front)
    omega_RR = omega_hover * 1.1  # Motor 4 (rear)

    _, _, tau_theta_pitch, _, _ = model.calcular_fuerzas_torques(
        omega_FR, omega_RL, omega_FL, omega_RR
    )
    print(f"\n[PITCH] Aplicando comando de pitch.")
    print(f"[PITCH] Torque de pitch: {tau_theta_pitch:.4f} (Debería ser > 0)")
    assert tau_theta_pitch > 0.0, "El torque de pitch debería ser positivo para inclinar hacia adelante"
    print("[PITCH] OK!")

    # 4. PRUEBA DE MOVIMIENTO EN YAW (GIRO SOBRE EJE Z)
    # Rotores CCW (1, 2) aumentan, CW (3, 4) disminuyen -> Yaw positivo
    omega_FR = omega_hover * 1.1  # Motor 1 (CCW)
    omega_RL = omega_hover * 1.1  # Motor 2 (CCW)
    omega_FL = omega_hover * 0.9  # Motor 3 (CW)
    omega_RR = omega_hover * 0.9  # Motor 4 (CW)

    _, _, _, tau_psi_yaw, _ = model.calcular_fuerzas_torques(
        omega_FR, omega_RL, omega_FL, omega_RR
    )
    print(f"\n[YAW] Aplicando comando de yaw.")
    print(f"[YAW] Torque de yaw: {tau_psi_yaw:.4f} (Debería ser > 0)")
    assert tau_psi_yaw > 0.0, "El torque de yaw debería ser positivo"
    print("[YAW] OK!")

    # 5. INTEGRACIÓN RK4 EN LA SIMULACIÓN
    # Reseteamos estado y corremos simulación con un comando de roll sostenido
    model.reset()
    dt = 0.01  # s
    pasos = 100 # 1 segundo de simulación
    print(f"\n[INTEGRACIÓN] Simulación de {pasos*dt}s con entrada de roll...")
    
    for _ in range(pasos):
        model.actualizar(omega_hover*0.95, omega_hover*1.05, omega_hover*1.05, omega_hover*0.95, dt)

    estado_final = model.get_estado()
    print(f"[INTEGRACIÓN] Estado final:")
    print(f"  Posición X, Y, Z : {estado_final[0]:.4f}, {estado_final[1]:.4f}, {estado_final[2]:.4f}")
    print(f"  Ángulos R, P, Y  : {estado_final[3]:.4f}, {estado_final[4]:.4f}, {estado_final[5]:.4f}")
    
    # Comprobar que el ángulo de roll (estado_final[3]) es positivo debido al torque de roll positivo aplicado
    assert estado_final[3] > 0.0, "El dron no se inclinó positivamente en roll"
    print("[INTEGRACIÓN] OK!")
    
    print("\n=== ¡TODAS LAS PRUEBAS UNITARIAS PASARON EXITOSAMENTE! ===")

if __name__ == "__main__":
    run_tests()
