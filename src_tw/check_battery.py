from pymavlink import mavutil
import time

print("Escuchando el puerto UDP 14551 para mensajes de batería...")
# Cambia 'udpin:0.0.0.0:14551' si transmites a otro puerto o por serial
master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')

print("Esperando el primer latido (heartbeat) del Pixhawk...")
master.wait_heartbeat()
print(f"¡Conectado al sistema {master.target_system}!")

# Solicitamos explícitamente el stream de estado extendido a 2 Hz
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
    2, 1
)
print("Solicitud de telemetría de batería enviada.")

paquetes_vistos = set()

try:
    while True:
        # Escuchamos cualquier mensaje que llegue
        msg = master.recv_match(blocking=True)
        if msg:
            tipo = msg.get_type()
            
            # Registrar si vemos paquetes nuevos
            if tipo not in paquetes_vistos:
                print(f"[NUEVO PAQUETE DETECTADO] -> {tipo}")
                paquetes_vistos.add(tipo)
            
            # Si el paquete es SYS_STATUS, extraemos el voltaje
            if tipo == "SYS_STATUS":
                voltaje_v = float(msg.voltage_battery) / 1000.0
                if voltaje_v > 0.0:
                    print(f"BATERÍA RECIBIDA (SYS_STATUS): {voltaje_v:.2f} V")
                else:
                    print("Se recibió SYS_STATUS pero el voltaje marca 0V (¿Sensor no configurado en Pixhawk?)")
                    
            # Si el paquete es BATTERY_STATUS, también lo leemos por si acaso
            elif tipo == "BATTERY_STATUS":
                voltaje_v = sum(msg.voltages) / 1000.0
                # En battery status, voltages es un arreglo con el voltaje de cada celda (o la suma en el primer índice)
                print(f"BATERÍA RECIBIDA (BATTERY_STATUS): {msg.voltages}")
except KeyboardInterrupt:
    print("\nCerrando...")
