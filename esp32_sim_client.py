#!/usr/bin/env python3
"""
esp32_sim_client.py

Simulador de cliente ESP32 (sin ROS2) que:
 - POST /esp32/send  (telemetría)
 - GET  /esp32/command (polling de comandos)
 
Uso:
    python3 esp32_sim_client.py --server http://192.168.40.76:8081 --client-id esp32-sim-01 --interval 2.0
"""

import requests
import time
import json
import threading
import argparse
import signal
import sys
import random
from datetime import datetime

# ---------- Config CLI ----------
parser = argparse.ArgumentParser(description="Simulador ESP32 cliente HTTP")
parser.add_argument('--server', '-s', default='http://127.0.0.1:8081', help='Base URL del servidor (ej: http://192.168.40.76:8081)')
parser.add_argument('--client-id', '-c', default='esp32-sim-01', help='Client-Id header para identificar al cliente')
parser.add_argument('--interval', '-i', type=float, default=2.0, help='Intervalo (seg) entre telemetría / polling')
parser.add_argument('--verbose', '-v', action='store_true', help='Salida verbosa (logs)')
args = parser.parse_args()

SERVER_BASE = args.server.rstrip('/')
CLIENT_ID = args.client_id
POLL_INTERVAL = max(0.2, args.interval)
VERBOSE = args.verbose

HEADERS = {
    'Content-Type': 'application/json',
    'Client-Id': CLIENT_ID
}

# ---------- Estado global y control graceful ----------
running = True

def signal_handler(sig, frame):
    global running
    print("\n[esp32-sim] Señal de terminación recibida. Saliendo...")
    running = False

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# ---------- Funciones utilitarias ----------
def log(msg, level='INFO'):
    if level == 'DEBUG' and not VERBOSE:
        return
    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print(f"[{now}] [{level}] {msg}")

def collect_telemetry():
    """
    Genera telemetría simulada. Reemplaza con lecturas reales si lo deseas.
    """
    # Simulación simple de sensores
    temp = round(20.0 + random.uniform(-2.0, 6.0), 2)   # °C
    hum  = round(30.0 + random.uniform(-5.0, 20.0), 2)  # %
    batt = round(3.3 - random.uniform(0.0, 0.6), 2)     # V

    telemetry = {
        "client_id": CLIENT_ID,
        "timestamp": time.time(),
        "human_timestamp": datetime.utcnow().isoformat() + "Z",
        "sensors": {
            "temperature_c": temp,
            "humidity_pct": hum,
            "battery_v": batt
        },
        "status": {
            "uptime_s": int(time.time() % 100000),
            "free_heap": random.randint(20000, 120000)
        }
    }
    return telemetry

def post_telemetry(telemetry, timeout=5):
    url = f"{SERVER_BASE}/esp32/send"
    try:
        r = requests.post(url, headers=HEADERS, json=telemetry, timeout=timeout)
        if r.ok:
            if VERBOSE:
                log(f"Telemetría enviada correctamente. Server response: {r.status_code}", 'DEBUG')
            return True, r.json() if r.text else {}
        else:
            log(f"POST telemetría falló: {r.status_code} - {r.text}", 'WARNING')
            return False, None
    except requests.RequestException as e:
        log(f"Exception POST telemetría: {e}", 'WARNING')
        return False, None

def get_command(timeout=5):
    url = f"{SERVER_BASE}/esp32/command"
    try:
        r = requests.get(url, headers=HEADERS, timeout=timeout)
        if r.ok:
            try:
                data = r.json()
            except ValueError:
                log("Respuesta GET /esp32/command no es JSON", 'WARNING')
                return False, None
            return True, data
        else:
            log(f"GET comando falló: {r.status_code} - {r.text}", 'WARNING')
            return False, None
    except requests.RequestException as e:
        log(f"Exception GET comando: {e}", 'WARNING')
        return False, None

def handle_command(cmd):
    """
    Ejecuta (simula) la acción recibida. Adapta este bloque a tus acciones reales.
    """
    try:
        cmd_id = cmd.get('id', 'unknown')
        action = cmd.get('action', 'noop')
        value = cmd.get('value', None)
        priority = cmd.get('priority', None)
        log(f"Recibido comando id={cmd_id} action={action} value={value} priority={priority}", 'INFO')

        # Simula diferentes acciones
        if action == 'reboot':
            log("Simulando reboot...", 'INFO')
            time.sleep(1.0)
            log("Reboot simulado completado.", 'INFO')
        elif action == 'led':
            # value puede ser {"pin": 2, "state": "on"}
            log(f"Simulación: cambiar LED a {value}", 'INFO')
        elif action == 'read_sensor':
            log("Simulando lectura de sensor por comando...", 'INFO')
            reading = {
                "sensor": "light",
                "value": random.randint(0, 1023)
            }
            log(f"Lectura simulada: {reading}", 'INFO')
            # opcional: enviar lectura resultante al servidor
            post_telemetry({
                "client_id": CLIENT_ID,
                "timestamp": time.time(),
                "command_result_for": cmd_id,
                "result": reading
            })
        else:
            log(f"Acción desconocida ('{action}'), no se ejecutó nada real.", 'INFO')
    except Exception as e:
        log(f"Error al manejar comando: {e}", 'ERROR')

# ---------- Loop principal (polling) ----------
def polling_loop():
    backoff = 1.0
    max_backoff = 30.0
    last_command_id = None

    while running:
        # 1) Preparar y enviar telemetría
        telemetry = collect_telemetry()
        ok, resp = post_telemetry(telemetry)
        if ok:
            backoff = 1.0  # reset backoff si va bien

        # 2) Consultar comandos en el servidor
        ok, data = get_command()
        if ok and isinstance(data, dict):
            if data.get('command_available'):
                cmd = data.get('command')
                if cmd:
                    cmd_id = cmd.get('id')
                    # Evitar procesar el mismo comando varias veces seguidas
                    if cmd_id != last_command_id:
                        last_command_id = cmd_id
                        handle_command(cmd)
                    else:
                        log("Comando repetido recibido; ignorado.", 'DEBUG')
            else:
                if VERBOSE:
                    log("No hay comandos pendientes.", 'DEBUG')

        # 3) Dormir intervalo (o backoff si hay fallos)
        # Si hubo muchos errores consecutivos (no ok), aumentar backoff
        # (aquí simplificamos: si cualquier llamada fallo, aumentamos)
        if not ok:
            backoff = min(max_backoff, backoff * 2)
            delay = backoff
            log(f"Errores detectados, aplicando backoff: {delay}s", 'WARNING')
        else:
            delay = POLL_INTERVAL

        slept = 0.0
        while running and slept < delay:
            time.sleep(0.1)
            slept += 0.1

    log("Polling loop terminado.", 'INFO')

# ---------- Interfaz mínima interactiva (opcional) ----------
def interactive_prompt():
    """
    Permite al usuario enviar telemetría o comandos manualmente mientras el polling corre en background.
    Comandos:
        send <json>   -> envía telemetría JSON al endpoint /esp32/send
        exit          -> salir
        help          -> muestra ayuda
    """
    global running 

    help_text = """
Comandos interactivos:
  send <json>    : enviar telemetría JSON, ej: send {"test":123}
  exit           : terminar el programa
  help           : mostrar esta ayuda
"""
    print(help_text)
    while running:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            # Si el usuario hace Ctrl+C dentro del prompt, salimos limpiamente
            running = False
            break

        if not line:
            continue

        if line == 'exit':
            log("Comando exit recibido. Saliendo...", 'INFO')
            # señalamos para terminar
            running = False
            break

        if line == 'help':
            print(help_text)
            continue

        if line.startswith('send '):
            payload_text = line[5:].strip()
            try:
                payload = json.loads(payload_text)
            except Exception as e:
                log(f"JSON inválido: {e}", 'ERROR')
                continue
            ok, _ = post_telemetry(payload)
            if ok:
                log("Telemetría enviada manualmente.", 'INFO')
            continue

        log("Comando no reconocido. Escribe 'help' para opciones.")


# ---------- Main ----------
if __name__ == '__main__':
    log(f"Iniciando simulador ESP32 -> server: {SERVER_BASE}  client_id: {CLIENT_ID}  interval: {POLL_INTERVAL}s", 'INFO')

    # Hilo polling (daemon)
    poll_thread = threading.Thread(target=polling_loop, daemon=True)
    poll_thread.start()

    # Hilo interactivo (no-daemon para poder cerrarlo bien), opcional:
    try:
        interactive_prompt()
    except Exception as e:
        log(f"Excepción en prompt interactivo: {e}", 'ERROR')

    # Esperar a que termine el polling loop
    poll_thread.join(timeout=2.0)
    log("Cliente simulador detenido.", 'INFO')
    sys.exit(0)
