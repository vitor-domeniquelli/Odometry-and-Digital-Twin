import RPi.GPIO as GPIO
import pygame
import time

# ======================
# SETUP DE PINOS
# ======================
MOTOR_ESQ_IN1 = 26 
MOTOR_ESQ_IN2 = 19
MOTOR_ESQ_EN  = 12

MOTOR_DIR_IN1 = 5
MOTOR_DIR_IN2 = 6 
MOTOR_DIR_EN  = 13

# Frequencia do PWM (1000 Hz e um bom padrao para motores DC)
FREQ_PWM = 1000

def setup_hardware():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Configura todos os pinos da Ponte H como saida
    pinos = [MOTOR_ESQ_IN1, MOTOR_ESQ_IN2, MOTOR_ESQ_EN, 
             MOTOR_DIR_IN1, MOTOR_DIR_IN2, MOTOR_DIR_EN]
    for pino in pinos:
        GPIO.setup(pino, GPIO.OUT)

    # Inicializa o PWM nos pinos de Enable (EN) com duty cycle inicial de 0% (parado)
    global pwm_esq, pwm_dir
    pwm_esq = GPIO.PWM(MOTOR_ESQ_EN, FREQ_PWM)
    pwm_dir = GPIO.PWM(MOTOR_DIR_EN, FREQ_PWM)
    pwm_esq.start(0)
    pwm_dir.start(0)

# ======================
# LOGICA DE MOVIMENTO
# ======================
def acionar_motor(lado, valor_eixo):
    """
    valor_eixo no pygame vai de -1.0 (analogico todo pra cima) 
    ate 1.0 (analogico todo pra baixo).
    """
    # ZONA MORTA: Evita que o robo ande sozinho se o analogico estiver um pouquinho solto
    ZONA_MORTA = 0.1 
    
    if abs(valor_eixo) < ZONA_MORTA:
        velocidade = 0
        frente = True # Irrelevante se a velocidade e 0
    else:
        # Define a direcao (Pra cima e negativo no Pygame)
        frente = valor_eixo < 0 
        
        # Converte o valor do eixo (0 a 1.0) para porcentagem de velocidade (0 a 100%)
        velocidade = min(abs(valor_eixo) * 100, 100)

    # Aplica os comandos de direcao e velocidade na Ponte H
    if lado == 'esq':
        GPIO.output(MOTOR_ESQ_IN1, GPIO.HIGH if frente else GPIO.LOW)
        GPIO.output(MOTOR_ESQ_IN2, GPIO.LOW if frente else GPIO.HIGH)
        pwm_esq.ChangeDutyCycle(velocidade)
    
    elif lado == 'dir':
        GPIO.output(MOTOR_DIR_IN1, GPIO.HIGH if frente else GPIO.LOW)
        GPIO.output(MOTOR_DIR_IN2, GPIO.LOW if frente else GPIO.HIGH)
        pwm_dir.ChangeDutyCycle(velocidade)

# ======================
# PROGRAMA PRINCIPAL
# ======================
def main():
    setup_hardware()
    
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("Nenhum controle detectado!")
        return

    controle = pygame.joystick.Joystick(0)
    controle.init()
    print(f"Controle conectado: {controle.get_name()}")

    try:
        while True:
            pygame.event.pump()

            eixo_esq_y = controle.get_axis(1) 
            eixo_dir_y = controle.get_axis(3) 

            # === LINHA NOVA PARA DEBUG ===
            print(f"Analogico Esq: {eixo_esq_y:5.2f} | Analogico Dir: {eixo_dir_y:5.2f}")

            acionar_motor('esq', eixo_esq_y)
            acionar_motor('dir', eixo_dir_y)

            time.sleep(0.1) # Aumentei um pouquinho a pausa para voce conseguir ler o terminal

    except KeyboardInterrupt:
        print("\nDesligando...")
    finally:
        pwm_esq.stop()
        pwm_dir.stop()
        GPIO.cleanup()
        pygame.quit()

if __name__ == "__main__":
    main()
