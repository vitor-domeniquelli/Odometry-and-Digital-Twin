import RPi.GPIO as GPIO
import time
import math
import threading
import csv
import matplotlib.pyplot as plt

# ======================
# PARÂMETROS FÍSICOS (CALIBRADOS)
# ======================
RAIO_RODA = 0.0301         # Raio efetivo calibrado
L = 0.375                 # Bitola nominal (Vamos calibrar este valor agora)

PULSOS_MOTOR = 40          
REDUCAO = 1.5              
PULSOS_POR_VOLTA_RODA = PULSOS_MOTOR * REDUCAO
DIST_POR_PULSO = (2 * math.pi * RAIO_RODA) / PULSOS_POR_VOLTA_RODA

TEMPO_MOVIMENTO = 4.0      # Segundos girando
VELOCIDADE_PWM = 80        # Ajuste se o robô não tiver torque para girar

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

# Variáveis globais
ticks_esq = 0
ticks_dir = 0
pose_x, pose_y, pose_theta = 0.0, 0.0, 0.0
history_x, history_y = [0.0], [0.0]
running = True

# ======================
# INTERRUPÇÕES
# ======================
def callback_esq(channel):
    global ticks_esq
    # Como o motor esquerdo vai pra frente, soma-se
    ticks_esq += 1

def callback_dir(channel):
    global ticks_dir
    # Como o motor direito vai pra trás (giro horário), os pulsos deveriam ser negativos
    # Mas como lemos apenas 1 canal (sem quadratura), tratamos a direção na matemática do encoder
    ticks_dir += 1

GPIO.add_event_detect(ENCODER_ESQ_PIN, GPIO.BOTH, callback=callback_esq)
GPIO.add_event_detect(ENCODER_DIR_PIN, GPIO.BOTH, callback=callback_dir)

# ======================
# CONTROLE DE MOTORES
# ======================
def girar_direita_in_place(velocidade):
    # Motor Esquerdo = Frente
    GPIO.output(MOTOR_ESQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_ESQ_IN2, GPIO.LOW)
    # Motor Direito = Trás
    GPIO.output(MOTOR_DIR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_DIR_IN2, GPIO.HIGH)
    
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
# ODOMETRIA E LOG
# ======================
def odometry_loop():
    global ticks_esq, ticks_dir, pose_x, pose_y, pose_theta, running
    last_ticks_esq, last_ticks_dir = 0, 0
    dt = 0.05
    
    with open('log_rotacao.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Tempo', 'X', 'Y', 'Theta'])
        start_time = time.time()
        
        while running:
            time.sleep(dt)
            # Como o robô está girando para a direita (sentido horário):
            # Esquerda vai pra frente (+), Direita vai pra trás (-)
            delta_esq = ticks_esq - last_ticks_esq
            delta_dir = -(ticks_dir - last_ticks_dir) # Inverte o sinal matematicamente
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
            writer.writerow([time.time() - start_time, pose_x, pose_y, math.degrees(pose_theta)])

# ======================
# MAIN
# ======================
def main():
    global running
    print("Iniciando Teste 2: Rotação Pura...")
    odo_thread = threading.Thread(target=odometry_loop)
    odo_thread.start()
    
    time.sleep(1)
    
    print(f"Girando por {TEMPO_MOVIMENTO} segundos a {VELOCIDADE_PWM}% de potência.")
    girar_direita_in_place(VELOCIDADE_PWM)
    time.sleep(TEMPO_MOVIMENTO)
    
    print("Parando motores.")
    parar_motores()
    
    time.sleep(0.5)
    running = False
    odo_thread.join()
    
    theta_graus = math.degrees(pose_theta)
    print(f"\nTicks Esquerda: {ticks_esq}")
    print(f"Ticks Direita : {ticks_dir}")
    print(f"Ângulo calculado final (Theta): {theta_graus:.2f} graus")
    
    plt.figure()
    plt.plot(history_x, history_y, 'r-', label='Ponto Central do Robô')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Teste 2: Rotação no Próprio Eixo')
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
