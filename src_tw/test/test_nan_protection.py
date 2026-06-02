"""
Test adicional que verifica la protección contra NaN.

Simula las condiciones exactas que causaron el problema del 29/05:
- PWM desbalanceados que generan torques grandes
- Simulación prolongada que antes causaba blow-up numérico
"""
import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')))

from gemelo_digital_qav250.modelo_euler_lagrange import DroneModel
from gemelo_digital_qav250.captura_pwm import pwm_a_omega

def test_nan_protection():
    """Reproduce el escenario del 29/05 que causaba NaN."""
    params = {
        "m": 0.650, "l": 0.125,
        "Ixx": 0.0058, "Iyy": 0.0058, "Izz": 0.0100,
        "k": 3.13e-05, "b": 7.50e-07, "Ir": 6.00e-05,
        "Ax": 0.0, "Ay": 0.0, "Az": 0.0
    }
    model = DroneModel(params)

    # PWM que causaron el problema: [1100, 1180, 1128, 1152]
    pwm_problematicos = [1100.0, 1180.0, 1128.0, 1152.0]
    omegas = [pwm_a_omega(p) for p in pwm_problematicos]
    
    print(f"PWM: {pwm_problematicos}")
    print(f"Omegas: {[f'{o:.1f}' for o in omegas]}")
    print()

    dt = 0.05  # 20 Hz
    nan_encontrado = False
    
    # Simular 10 segundos (200 pasos) — antes crasheaba en ~10 pasos
    for i in range(200):
        estado = model.actualizar(*omegas, dt)
        
        if np.any(np.isnan(estado)):
            nan_encontrado = True
            print(f"[FALLO] NaN detectado en paso {i}")
            break
        
        if i % 20 == 0:  # Cada segundo
            print(f"  t={i*dt:.1f}s  |  pos=({estado[0]:.3f}, {estado[1]:.3f}, {estado[2]:.3f})  "
                  f"|  RPY=({np.degrees(estado[3]):.1f}°, {np.degrees(estado[4]):.1f}°, {np.degrees(estado[5]):.1f}°)")

    if not nan_encontrado:
        print(f"\n✅ PROTECCIÓN NaN FUNCIONA: 200 pasos sin NaN con PWM problemáticos")
    else:
        print(f"\n❌ FALLO: NaN apareció aún con protección")
        return False

    # Test 2: Simular condiciones extremas (PWM muy desbalanceados)
    print("\n--- Test 2: PWM extremadamente desbalanceados ---")
    model.reset()
    pwm_extremos = [1000.0, 2000.0, 1000.0, 2000.0]
    omegas_ext = [pwm_a_omega(p) for p in pwm_extremos]
    
    for i in range(200):
        estado = model.actualizar(*omegas_ext, dt)
        if np.any(np.isnan(estado)):
            nan_encontrado = True
            print(f"[FALLO] NaN en paso {i} con PWM extremos")
            break
        if i % 40 == 0:
            print(f"  t={i*dt:.1f}s  |  RPY=({np.degrees(estado[3]):.1f}°, "
                  f"{np.degrees(estado[4]):.1f}°, {np.degrees(estado[5]):.1f}°)")

    if not nan_encontrado:
        print(f"✅ PROTECCIÓN NaN FUNCIONA: PWM extremos manejados correctamente")
    
    return not nan_encontrado


if __name__ == "__main__":
    print("=" * 65)
    print("  TEST DE PROTECCIÓN CONTRA NaN — Escenario del 29/05")
    print("=" * 65)
    exito = test_nan_protection()
    print()
    if exito:
        print("=== ¡TODOS LOS TESTS DE NaN PASARON! ===")
    else:
        print("=== ALGUNOS TESTS FALLARON ===")
        sys.exit(1)
