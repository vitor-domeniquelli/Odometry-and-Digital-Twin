import RPi.GPIO as GPIO
import time
import math
import threading
import csv
import matplotlib.pyplot as plt

# ======================
# PARÂMETROS FÍSICOS (CALIBRADOS)
# ======================
RAIO_RODA = 0.0301         # Raio efetivo 
L = 0.326                  # Bitola efetiva calibrada (Leff)

PULSOS_MOTOR = 40          
REDUCAO = 1.5              
PULSOS_POR_VOLTA_RODA = PULSOS_MOTOR * REDUCAO
DIST_POR_PULSO = (2 * math.pi * RAIO_RODA) / PULSOS_POR_VOLTA_RODA

# Parâmetros de percurso
TEMPO_RETO = 1.0
VEL_PWM_RETO = 60
VEL_PWM_GIRO = 80

# ======================
# SETUP GPIO
# ======================
ENCODER_ESQ_PIN = 16
ENCODER_DIR_PIN = 20

MOTOR_ESQ_IN1 = 6
MOTOR_ESQ_IN2 = 5
MOTOR_ESQ_EN  = 12

MOTOR_DIR_IN1 = 19
MOTOR_DIR_IN2 = 26 
MOTOR_DIR_EN  = 13

GPIO.setmode(GPIO.BCM)
GPIO.setup([ENCODER_ESQ_PIN, ENCODER_DIR_PIN], GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup([MOTOR_ESQ_IN1, MOTOR_ESQ_IN2, MOTOR_ESQ_EN, MOTOR_DIR_IN1, MOTOR_DIR_IN2, MOTOR_DIR_EN], GPIO.OUT)

pwm_esq = GPIO.PWM(MOTOR_ESQ_EN, 100)
pwm_dir = GPIO.PWM(MOTOR_DIR_EN, 100)
pwm_esq.start(0)
pwm_dir.start(0)

ticks_esq, ticks_dir = 0, 0
pose_x, pose_y, pose_theta = 0.0, 0.0, 0.0
history_x, history_y = [0.0], [0.0]

running = True
estado_movimento = 'PARADO'

# ======================
# INTERRUPÇÕES
# ======================
def callback_esq(channel):
    global ticks_esq
    ticks_esq += 1

def callback_dir(channel):
    global ticks_dir
    ticks_dir += 1

GPIO.add_event_detect(ENCODER_ESQ_PIN, GPIO.BOTH, callback=callback_esq)
GPIO.add_event_detect(ENCODER_DIR_PIN, GPIO.BOTH, callback=callback_dir)

# ======================
# CONTROLE DE MOTORES
# ======================
def motores_para_frente(velocidade):
    global estado_movimento
    estado_movimento = 'RETO'
    
    GPIO.output(MOTOR_ESQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_ESQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DIR_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_DIR_IN2, GPIO.LOW)
    pwm_esq.ChangeDutyCycle(velocidade)
    pwm_dir.ChangeDutyCycle(velocidade)

def girar_direita_in_place(velocidade):
    global estado_movimento
    estado_movimento = 'GIRO_DIR'
    
    GPIO.output(MOTOR_ESQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_ESQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DIR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_DIR_IN2, GPIO.HIGH)
    pwm_esq.ChangeDutyCycle(velocidade)
    pwm_dir.ChangeDutyCycle(velocidade)

def parar_motores():
    global estado_movimento
    estado_movimento = 'PARADO'
    
    pwm_esq.ChangeDutyCycle(0)
    pwm_dir.ChangeDutyCycle(0)
    GPIO.output(MOTOR_ESQ_IN1, GPIO.LOW)
    GPIO.output(MOTOR_ESQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DIR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_DIR_IN2, GPIO.LOW)

# ======================
# ODOMETRIA E LOG
# ======================
def odometry_loop():
    global ticks_esq, ticks_dir, pose_x, pose_y, pose_theta, running
    last_ticks_esq, last_ticks_dir = 0, 0
    dt = 0.05
    
    with open('log_quadrado.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Tempo', 'X', 'Y', 'Theta'])
        start_time = time.time()
        
        while running:
            time.sleep(dt)
            
            delta_esq = ticks_esq - last_ticks_esq
            delta_dir_bruto = ticks_dir - last_ticks_dir
            last_ticks_esq, last_ticks_dir = ticks_esq, ticks_dir
            
            # Ajuste de sinal baseado na ação atual
            delta_dir = -delta_dir_bruto if estado_movimento == 'GIRO_DIR' else delta_dir_bruto
            
            delta_s_esq = delta_esq * DIST_POR_PULSO
            delta_s_dir = delta_dir * DIST_POR_PULSO
            
            delta_s = (delta_s_dir + delta_s_esq) / 2.0
            delta_theta = (delta_s_dir - delta_s_esq) / L
            
            pose_x += delta_s * math.cos(pose_theta + (delta_theta / 2.0))
            pose_y += delta_s * math.sin(pose_theta + (delta_theta / 2.0))
            pose_theta += delta_theta
            
            history_x.append(pose_x)
            history_y.append(pose_y)
            writer.writerow([time.time() - start_time, pose_x, pose_y, math.degrees(pose_theta)])

# ======================
# LÓGICA DE MOVIMENTO
# ======================
def virar_90_graus_direita():
    global running
    theta_inicial = pose_theta
    # Alvo é -90 graus, convertido para radianos
    alvo_theta = theta_inicial - math.radians(90)
    
    girar_direita_in_place(VEL_PWM_GIRO)
    
    # Bloqueia até o robô atingir ou ultrapassar o ângulo alvo
    while pose_theta > alvo_theta and running:
        time.sleep(0.01)
        
    parar_motores()
    print(f"Giro concluído. Ângulo atual: {math.degrees(pose_theta):.1f} graus")

# ======================
# MAIN
# ======================
def main():
    global running
    print("Iniciando Teste 3: Trajetória Quadrada...")
    odo_thread = threading.Thread(target=odometry_loop)
    odo_thread.start()
    
    time.sleep(1)
    
    for lado in range(1, 5):
        print(f"\n--- Lado {lado} ---")
        
        print("Andando para frente...")
        motores_para_frente(VEL_PWM_RETO)
        time.sleep(TEMPO_RETO)
        parar_motores()
        time.sleep(0.5) 
        
        if lado < 4:
            print("Girando 90 graus...")
            virar_90_graus_direita()
            time.sleep(0.5)
            
    running = False
    odo_thread.join()
    
    print(f"\n=== RESULTADOS FINAIS ===")
    print(f"X Final: {pose_x:.3f} m")
    print(f"Y Final: {pose_y:.3f} m")
    print(f"Theta Final: {math.degrees(pose_theta):.2f} graus")
    
    erro_dist = math.sqrt(pose_x**2 + pose_y**2)
    print(f"Erro absoluto de fechamento (Odometria): {erro_dist:.3f} m")
    
    plt.figure()
    plt.plot(history_x, history_y, 'b-', label='Odometria Calculada')
    plt.plot(0, 0, 'go', label='Início (0,0)')
    plt.plot(pose_x, pose_y, 'ro', label='Fim (Calculado)')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Teste 3: Trajetória Quadrada (Dead-reckoning)')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        parar_motores()
        pwm_esq.stop()
        pwm_dir.stop()
        GPIO.cleanup()
