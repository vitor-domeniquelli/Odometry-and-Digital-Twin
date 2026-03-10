import RPi.GPIO as GPIO
import time
import math
import threading
import csv
import matplotlib.pyplot as plt

# ======================
# PARÂMETROS FÍSICOS E DE TESTE
# ======================
RAIO_RODA = 0.030    # Metros (Ajustar após este teste)
L = 0.130            # Metros (Bitola)
PULSOS_POR_VOLTA = 40
DIST_POR_PULSO = (2 * math.pi * RAIO_RODA) / PULSOS_POR_VOLTA

TEMPO_MOVIMENTO = 3.0  # Tempo em segundos que o robô andará para frente
VELOCIDADE_PWM = 60    # Duty cycle (0 a 100%) - Ajuste para evitar escorregamento

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

# Configuração do PWM (Frequência de 100Hz)
pwm_esq = GPIO.PWM(MOTOR_ESQ_EN, 100)
pwm_dir = GPIO.PWM(MOTOR_DIR_EN, 100)
pwm_esq.start(0)
pwm_dir.start(0)

# Variáveis globais
ticks_esq = 0
ticks_dir = 0
pose_x = 0.0
pose_y = 0.0
pose_theta = 0.0
history_x = [0.0]
history_y = [0.0]
running = True

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
    GPIO.output(MOTOR_ESQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_ESQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DIR_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_DIR_IN2, GPIO.LOW)
    
    pwm_esq.ChangeDutyCycle(velocidade)
    pwm_dir.ChangeDutyCycle(velocidade)

def parar_motores():
    pwm_esq.ChangeDutyCycle(0)
    pwm_dir.ChangeDutyCycle(0)
    GPIO.output(MOTOR_ESQ_IN1, GPIO.LOW)
    GPIO.output(MOTOR_ESQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DIR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_DIR_IN2, GPIO.LOW)

# ======================
# ODOMETRIA E LOG (THREAD)
# ======================
def odometry_loop():
    global ticks_esq, ticks_dir, pose_x, pose_y, pose_theta, running
    last_ticks_esq, last_ticks_dir = 0, 0
    dt = 0.05  # 20 Hz para maior resolução de log
    
    with open('log_transla_pura.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Tempo', 'X', 'Y', 'Theta'])
        start_time = time.time()
        
        while running:
            time.sleep(dt)
            delta_esq = ticks_esq - last_ticks_esq
            delta_dir = ticks_dir - last_ticks_dir
            last_ticks_esq, last_ticks_dir = ticks_esq, ticks_dir
            
            delta_s_esq = delta_esq * DIST_POR_PULSO
            delta_s_dir = delta_dir * DIST_POR_PULSO
            
            delta_s = (delta_s_dir + delta_s_esq) / 2.0
            delta_theta = (delta_s_dir - delta_s_esq) / L
            
            pose_x += delta_s * math.cos(pose_theta + (delta_theta / 2.0))
            pose_y += delta_s * math.sin(pose_theta + (delta_theta / 2.0))
            pose_theta += delta_theta
            
            history_x.append(pose_x)
            history_y.append(pose_y)
            writer.writerow([time.time() - start_time, pose_x, pose_y, pose_theta])

# ======================
# MAIN
# ======================
def main():
    global running
    print("Iniciando odometria...")
    odo_thread = threading.Thread(target=odometry_loop)
    odo_thread.start()
    
    time.sleep(1) # Aguarda a thread iniciar
    
    print(f"Acionando motores por {TEMPO_MOVIMENTO} segundos a {VELOCIDADE_PWM}% de potência.")
    motores_para_frente(VELOCIDADE_PWM)
    time.sleep(TEMPO_MOVIMENTO)
    
    print("Parando motores.")
    parar_motores()
    
    time.sleep(0.5) # Aguarda acomodação mecânica final
    running = False
    odo_thread.join()
    
    print(f"Distância calculada final (X): {pose_x:.4f} m")
    print(f"Distância calculada final (Y): {pose_y:.4f} m")
    print(f"Ângulo calculado final (Theta): {math.degrees(pose_theta):.2f} °")
    
    # Plotagem
    plt.figure()
    plt.plot(history_x, history_y, 'b-', label='Trajetória Calculada')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Teste 1: Translação Pura (Pós-processamento)')
    plt.grid(True)
    plt.axis('equal') # Mantém a proporção real do movimento
    plt.legend()
    plt.show()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrompido.")
    finally:
        running = False
        parar_motores()
        pwm_esq.stop()
        pwm_dir.stop()
        GPIO.cleanup()