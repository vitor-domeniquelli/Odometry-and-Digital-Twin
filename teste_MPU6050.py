import smbus
import time

# ======================
# CONFIGURAÇÕES
# ======================

BUS_NUMBER = 1
MPU_ADDR = 0x68

# Registradores importantes
PWR_MGMT_1 = 0x6B

ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

# Escalas padrão
ACCEL_SCALE = 16384.0   # LSB/g  (±2g)
GYRO_SCALE  = 131.0     # LSB/(°/s) (±250°/s)

# ======================
# FUNÇÕES AUXILIARES
# ======================

def ler_palavra(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low  = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low

    # Converte para signed 16 bits
    if value >= 0x8000:
        value = -((65535 - value) + 1)

    return value

# ======================
# INICIALIZAÇÃO
# ======================

bus = smbus.SMBus(BUS_NUMBER)

# Acorda o MPU-6050
bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)

print("Teste MPU-6050 iniciado")
time.sleep(1)

# ======================
# LOOP DE LEITURA
# ======================

try:
    while True:
        # -------- Acelerômetro --------
        ax = ler_palavra(bus, MPU_ADDR, ACCEL_XOUT_H)     / ACCEL_SCALE
        ay = ler_palavra(bus, MPU_ADDR, ACCEL_XOUT_H + 2) / ACCEL_SCALE
        az = ler_palavra(bus, MPU_ADDR, ACCEL_XOUT_H + 4) / ACCEL_SCALE

        # -------- Giroscópio --------
        gx = ler_palavra(bus, MPU_ADDR, GYRO_XOUT_H)     / GYRO_SCALE
        gy = ler_palavra(bus, MPU_ADDR, GYRO_XOUT_H + 2) / GYRO_SCALE
        gz = ler_palavra(bus, MPU_ADDR, GYRO_XOUT_H + 4) / GYRO_SCALE

        print(
            f"ACC [g]  X:{ax:+.2f} Y:{ay:+.2f} Z:{az:+.2f} | "
            f"GYRO [°/s] X:{gx:+.1f} Y:{gy:+.1f} Z:{gz:+.1f}"
        )

        time.sleep(0.1)  # 10 Hz

except KeyboardInterrupt:
    print("\nTeste encerrado")
