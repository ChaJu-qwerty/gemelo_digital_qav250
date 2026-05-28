import socket
from pymavlink import mavutil
import time

def main():
    udp_ip = "0.0.0.0"
    udp_port = 14551
    connection_string = f"udpin:{udp_ip}:{udp_port}"
    
    print("==================================================")
    # Get local IPs
    hostname = socket.gethostname()
    try:
        local_ips = socket.gethostbyname_ex(hostname)[2]
    except Exception:
        local_ips = []
    print(f"Dirección IP local de esta computadora (Laptop B):")
    for ip in local_ips:
        if not ip.startswith("127."):
            print(f"  - {ip}")
    print(f"Escuchando tráfico MAVLink UDP en el puerto {udp_port}...")
    print("==================================================")

    try:
        mav = mavutil.mavlink_connection(connection_string)
    except Exception as e:
        print(f"Error al abrir el puerto UDP: {e}")
        return

    print("Esperando paquetes MAVLink... (Presiona Ctrl+C para salir)")
    last_print = 0
    seen_systems = {}

    try:
        while True:
            # Espera cualquier mensaje MAVLink
            msg = mav.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                if time.time() - last_print > 5:
                    print("No se reciben datos... Verifica el reenvío UDP en QGroundControl.")
                    last_print = time.time()
                continue

            # Obtener metadatos del mensaje
            msg_type = msg.get_type()
            src_system = msg.get_srcSystem()
            src_component = msg.get_srcComponent()
            
            key = (src_system, src_component)
            if key not in seen_systems:
                seen_systems[key] = set()
            seen_systems[key].add(msg_type)

            # Imprimir resumen de lo que se recibe cada 2 segundos
            if time.time() - last_print > 2:
                print(f"\n[{time.strftime('%H:%M:%S')}] Sistemas MAVLink detectados activos:")
                for (sys_id, comp_id), msg_types in seen_systems.items():
                    sys_name = "Desconocido"
                    if sys_id == 255 or sys_id == 0:
                        sys_name = "QGroundControl / GCS"
                    elif sys_id == 1:
                        sys_name = "Pixhawk (Autopiloto)"
                    
                    has_servo = "SERVO_OUTPUT_RAW" in msg_types
                    print(f"  * System ID: {sys_id} | Component ID: {comp_id} ({sys_name})")
                    print(f"    - Mensajes recibidos: {list(msg_types)[:8]}... (Total: {len(msg_types)} tipos)")
                    print(f"    - ¿Envia PWM motores (SERVO_OUTPUT_RAW)?: {'SÍ' if has_servo else 'NO'}")
                
                # Limpiar temporalmente para ver actividad reciente
                seen_systems.clear()
                last_print = time.time()

    except KeyboardInterrupt:
        print("\nPrueba de diagnóstico terminada por el usuario.")
    finally:
        mav.close()

if __name__ == "__main__":
    main()
