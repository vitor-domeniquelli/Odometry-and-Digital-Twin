#!/usr/bin/env python3
import os
import sys
import subprocess
import time

MAC = '0C:B8:15:5A:9D:56'
WS  = os.path.expanduser('~/artigo_ieee_ws')

def step(n, total, msg):
    print(f'[{n}/{total}] {msg}', end='... ', flush=True)

def ok():
    print('OK')

def fail(msg):
    print(f'ERRO\n[ERRO] {msg}')
    sys.exit(1)

# ── 1. Bluetooth ───────────────────────────────────────────────────────────────
step(1, 6, 'Iniciando Bluetooth')
subprocess.run(['sudo', 'systemctl', 'start', 'bluetooth'])
ok()

# ── 2. Libera rfcomm anterior ──────────────────────────────────────────────────
step(2, 6, 'Liberando rfcomm anterior')
subprocess.run(['sudo', 'rfcomm', 'release', '0'], stderr=subprocess.DEVNULL)
ok()

# ── 3. Bind com o ESP32 ────────────────────────────────────────────────────────
step(3, 6, f'Criando bind rfcomm0 → {MAC}')
subprocess.run(['sudo', 'rfcomm', 'bind', '0', MAC, '1'])
ok()

# ── 4. Verifica /dev/rfcomm0 ──────────────────────────────────────────────────
step(4, 6, 'Verificando /dev/rfcomm0')
time.sleep(2)
if not os.path.exists('/dev/rfcomm0'):
    fail('/dev/rfcomm0 não criado. Verifique se o robô está ligado e pareado.')
ok()

# ── 5. Workspace ROS2 ─────────────────────────────────────────────────────────
step(5, 6, 'Verificando workspace ROS2')
setup_bash = os.path.join(WS, 'install', 'setup.bash')
if not os.path.exists(setup_bash):
    fail(f'Workspace não compilada. Execute:\n  cd {WS} && colcon build --packages-select robot_digital_twin')
ok()

# ── 6. Pasta de logs ──────────────────────────────────────────────────────────
step(6, 6, 'Criando pasta ~/ros_logs')
os.makedirs(os.path.expanduser('~/ros_logs'), exist_ok=True)
ok()

# ── Instruções finais ─────────────────────────────────────────────────────────
print()
print('Ambiente pronto. Para lançar o projeto:')
print()
print(f'  source {setup_bash}')
print( '  ros2 launch robot_digital_twin robot_launch.py')
print()
