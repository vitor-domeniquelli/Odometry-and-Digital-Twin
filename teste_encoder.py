import RPi.GPIO as GPIO
import time
import math

# ======================
# SETUP -- Ajustar os pinos conforme a montagem do hardware
# ======================

GPIO.setmode(GPIO.BCM)

# Encoders
ENCODER_ESQ_PIN = 16
ENCODER_DIR_PIN = 20

# Ponte H - esquerda
MOTOR_ESQ_IN1 = 6
MOTOR_ESQ_IN2 = 5
MOTOR_ESQ_EN  = 12

# Ponte H - direita
MOTOR_DIR_IN1 = 19
MOTOR_DIR_IN2 = 26 
MOTOR_DIR_EN  = 13

# Parâmetros físicos
PULSOS_POR_VOLTA = 40
DIAMETRO_RODA = 0.060
TEMPO_MOVIMENTO = 2

# ======================
# FUNÇÕES DE MOTOR
# ======================

def motores_para_frente():
    GPIO.output(MOTOR_ESQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_ESQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_ESQ_EN, GPIO.HIGH)

    GPIO.output(MOTOR_DIR_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_DIR_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DIR_EN, GPIO.HIGH)

def parar_motores():
    GPIO.output(MOTOR_ESQ_EN, GPIO.LOW)
    GPIO.output(MOTOR_DIR_EN, GPIO.LOW)

# ======================
# MAIN
# ======================

def main():
    # Setup motores
    GPIO.setup(MOTOR_ESQ_IN1, GPIO.OUT)
    GPIO.setup(MOTOR_ESQ_IN2, GPIO.OUT)
    GPIO.setup(MOTOR_ESQ_EN,  GPIO.OUT)

    GPIO.setup(MOTOR_DIR_IN1, GPIO.OUT)
    GPIO.setup(MOTOR_DIR_IN2, GPIO.OUT)
    GPIO.setup(MOTOR_DIR_EN,  GPIO.OUT)

    # Setup encoders
    GPIO.setup(ENCODER_ESQ_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_DIR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    pulsos_esq = 0
    pulsos_dir = 0

    estado_esq_ant = GPIO.input(ENCODER_ESQ_PIN)
    estado_dir_ant = GPIO.input(ENCODER_DIR_PIN)

    print("Teste de encoder")
    time.sleep(1)

    print("Robô andando para frente por 2 segundos...")
    inicio = time.time()
    motores_para_frente()

    while time.time() - inicio < TEMPO_MOVIMENTO:
        estado_esq = GPIO.input(ENCODER_ESQ_PIN)
        estado_dir = GPIO.input(ENCODER_DIR_PIN)

        if estado_esq != estado_esq_ant:
            pulsos_esq += 1

        if estado_dir != estado_dir_ant:
            pulsos_dir += 1

        estado_esq_ant = estado_esq
        estado_dir_ant = estado_dir

        time.sleep(0.0005)  # 0.5 ms

        tempo_total = time.time() - inicio

    parar_motores()

    # Cálculo de distância
    circunferencia = math.pi * DIAMETRO_RODA
    dist_por_pulso = circunferencia / PULSOS_POR_VOLTA

    dist_esq = pulsos_esq * dist_por_pulso
    dist_dir = pulsos_dir * dist_por_pulso
    dist_media = (dist_esq + dist_dir) / 2.0

    print("\nResultados:")
    print(f" Tempo total     : {tempo_total:.2f} s")
    print(f" Pulsos esquerda: {pulsos_esq}")
    print(f" Pulsos direita : {pulsos_dir}")
    print(f" Distância esquerda: {dist_esq:.3f} m")
    print(f" Distância direita : {dist_dir:.3f} m")
    print(f" Distância média   : {dist_media:.3f} m")

    GPIO.cleanup()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrompido pelo usuario")
        GPIO.cleanup()
