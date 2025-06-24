# ----------------------Sensores----------------------
from machine import I2C, Pin, UART
import time
import math
import dht
from micropyGPS import MicropyGPS

my_gps = MicropyGPS()

gps_serial = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))

sensor = dht.DHT11(Pin(22))

i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)


class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        self.i2c.writeto_mem(self.addr, 0x6B, b"\x00")
    def read_accel(self):
        data = self.i2c.readfrom_mem(self.addr, 0x3B, 6)
        ax = self._bytes_to_int(data[0:2])
        ay = self._bytes_to_int(data[2:4])
        az = self._bytes_to_int(data[4:6])
        return ax / 16384, ay / 16384, az / 16384
    def read_gyro(self):
        data = self.i2c.readfrom_mem(self.addr, 0x43, 6)
        gx = self._bytes_to_int(data[0:2])
        gy = self._bytes_to_int(data[2:4])
        gz = self._bytes_to_int(data[4:6])
        return gx / 131, gy / 131, gz / 131
    def _bytes_to_int(self, b):
        value = b[0] << 8 | b[1]
        if value > 32767:
            value -= 65536
        return value
    def read_temperature(self):
        # Lee 2 bytes empezando desde el registro 0x41 (TEMP_OUT_H)
        data = self.i2c.readfrom_mem(self.addr, 0x41, 2)
        raw_temp = self._bytes_to_int(data)
        # Aplicar la fórmula de conversión a Celsius
        temp_c = (raw_temp / 340.0) + 36.53
        return temp_c


class HMC5883L:
    def __init__(self, i2c, addr=0x1E):
        self.i2c = i2c
        self.addr = addr
        self.i2c.writeto_mem(self.addr, 0x00, b"\x70")
        self.i2c.writeto_mem(self.addr, 0x01, b"\xa0")
        self.i2c.writeto_mem(self.addr, 0x02, b"\x00")
    def read(self):
        data = self.i2c.readfrom_mem(self.addr, 0x03, 6)
        x = self._to_int(data[0:2])
        z = self._to_int(data[2:4])
        y = self._to_int(data[4:6])
        return x, y, z
    def _to_int(self, b):
        value = b[0] << 8 | b[1]
        if value > 32767:
            value -= 65536
        return value


class BMP180:
    def __init__(self, i2c, addr=0x77):
        self.i2c = i2c
        self.addr = addr
        self.AC1 = self._read_s16(0xAA)
        self.AC2 = self._read_s16(0xAC)
        self.AC3 = self._read_s16(0xAE)
        self.AC4 = self._read_u16(0xB0)
        self.AC5 = self._read_u16(0xB2)
        self.AC6 = self._read_u16(0xB4)
        self.B1 = self._read_s16(0xB6)
        self.B2 = self._read_s16(0xB8)
        self.MB = self._read_s16(0xBA)
        self.MC = self._read_s16(0xBC)
        self.MD = self._read_s16(0xBE)
    def _read_u16(self, reg):
        data = self.i2c.readfrom_mem(self.addr, reg, 2)
        return data[0] << 8 | data[1]
    def _read_s16(self, reg):
        result = self._read_u16(reg)
        if result > 32767:
            result -= 65536
        return result
    def read_raw_temp(self):
        self.i2c.writeto_mem(self.addr, 0xF4, b"\x2e")
        time.sleep(0.005)
        data = self.i2c.readfrom_mem(self.addr, 0xF6, 2)
        return data[0] << 8 | data[1]
    def read_raw_pressure(self, oss=0):
        self.i2c.writeto_mem(self.addr, 0xF4, bytes([0x34 + (oss << 6)]))
        if oss == 0:
            time.sleep(0.005)
        elif oss == 1:
            time.sleep(0.008)
        elif oss == 2:
            time.sleep(0.014)
        elif oss == 3:
            time.sleep(0.026)
        data = self.i2c.readfrom_mem(self.addr, 0xF6, 3)
        raw = ((data[0] << 16) + (data[1] << 8) + data[2]) >> (8 - oss)
        return raw
    def read_temperature(self):
        UT = self.read_raw_temp()
        X1 = ((UT - self.AC6) * self.AC5) >> 15
        X2 = (self.MC << 11) // (X1 + self.MD)
        B5 = X1 + X2
        temp = ((B5 + 8) >> 4) / 10.0
        return temp
    def read_pressure(self, oss=0):
        UT = self.read_raw_temp()
        UP = self.read_raw_pressure(oss)
        X1 = ((UT - self.AC6) * self.AC5) >> 15
        X2 = (self.MC << 11) // (X1 + self.MD)
        B5 = X1 + X2
        B6 = B5 - 4000
        X1 = (self.B2 * ((B6 * B6) >> 12)) >> 11
        X2 = (self.AC2 * B6) >> 11
        X3 = X1 + X2
        B3 = (((self.AC1 * 4 + X3) << oss) + 2) >> 2
        X1 = (self.AC3 * B6) >> 13
        X2 = (self.B1 * ((B6 * B6) >> 12)) >> 16
        X3 = ((X1 + X2) + 2) >> 2
        B4 = (self.AC4 * (X3 + 32768)) >> 15
        B7 = (UP - B3) * (50000 >> oss)
        if B7 < 0x80000000:
            p = (B7 * 2) // B4
        else:
            p = (B7 // B4) * 2
        X1 = (p >> 8) * (p >> 8)
        X1 = (X1 * 3038) >> 16
        X2 = (-7357 * p) >> 16
        p = p + ((X1 + X2 + 3791) >> 4)
        return p


# ----------------------Transmicion---------------------
import ustruct
import utime
from machine import Pin, SPI
from nrf24l01 import NRF24L01  # Asegúrate de tener este archivo en el Pico
import nrf24l01 as nr

mpu = MPU6050(i2c)
# mag = HMC5883L(i2c)
bmp = BMP180(i2c)


# -----------------------Recolectar datos-----------------------
# Asumiendo que NRF24L01, machine, y las clases de sensores están importadas
# Asumiendo que nrf, mpu, bmp, sensor, gps_serial, my_gps están inicializados globalmente o pasados

# --- Constantes de Escala ---
# Usar el mismo factor al empaquetar y desempaquetar
SCALE_FACTOR_ACCEL = 1000.0
SCALE_FACTOR_GYRO = 100.0
SCALE_FACTOR_TEMP = 100.0

# Define el formato del struct para el payload EXACTAMENTE igual en sender y receiver
PAYLOAD_FORMAT = '<h6hhihBffB'
EXPECTED_PAYLOAD_SIZE = ustruct.calcsize(PAYLOAD_FORMAT) # Debería ser 32

# Asegúrate de inicializar el NRF con el tamaño correcto en tu setup()
# nrf = NRF24L01(spi, csn, ce, payload_size=EXPECTED_PAYLOAD_SIZE) # payload_size=32

def safe_read(sensor_func, default_value=(0.0,) * 7): # Ajusta el número de valores por defecto
    try:
        return sensor_func()
    except Exception as e:
        print(f"Error reading {sensor_func.__name__}: {e}")
        return default_value

def read_mpu_all():
    ax, ay, az = mpu.read_accel()
    gx, gy, gz = mpu.read_gyro()
    temp_mpu = mpu.read_temperature()
    return ax, ay, az, gx, gy, gz, temp_mpu

def read_bmp_all():
    # Asegúrate de que tu librería BMP devuelva estos valores
    # Puede que necesites llamar a diferentes funciones
    temp_bmp = bmp.read_temperature() # O bmp.read_temperature()
    presion = bmp.read_pressure()    # O bmp.read_pressure()
    return temp_bmp, presion

def read_dht_all():
    try:
        sensor.measure()
        temp_dht = sensor.temperature()
        hum = sensor.humidity()
        return temp_dht, hum
    except Exception as e:
        print(f"Error DHT: {e}")
        return float('nan'), float('nan') # Valores por defecto

def read_gps_all():
    lat_decimal, lon_decimal, sats, speed_knots = 0.0, 0.0, 0, 0.0
    speed_mps = 0.0
    try:
        # Lee suficientes datos para potencialmente obtener una sentencia completa
        # Ajusta el tamaño del buffer si es necesario
        buffer_size = 256
        data = gps_serial.read(buffer_size)
        if data:
            for byte_val in data:
                stat = my_gps.update(chr(byte_val))
                # Puedes comprobar 'stat' si tu librería lo devuelve para saber si hay nueva sentencia

        # Intenta obtener los valores decimales directamente
        if my_gps.valid and my_gps.latitude and my_gps.longitude:
            # Asumiendo que MicropyGPS > 0.3.1 que tiene latitude_decimal() etc.
            # Si no, necesitarás calcularlos desde my_gps.latitude y my_gps.longitude (listas [grados, minutos, N/S])
            # lat_decimal = my_gps.latitude[0] + (my_gps.latitude[1] / 60.0)
            # if my_gps.latitude[2] == 'S': lat_decimal *= -1
            # lon_decimal = my_gps.longitude[0] + (my_gps.longitude[1] / 60.0)
            # if my_gps.longitude[2] == 'W': lon_decimal *= -1

            # Usa los métodos si existen, si no, calcula como arriba
            lat_decimal = my_gps.latitude_decimal if hasattr(my_gps, 'latitude_decimal') else 0.0
            lon_decimal = my_gps.longitude_decimal if hasattr(my_gps, 'longitude_decimal') else 0.0
            sats = my_gps.satellites_in_use
            speed_knots = my_gps.speed_knots() if hasattr(my_gps, 'speed_knots') else [0.0] # devuelve lista speed[2] = knots

            # Convierte velocidad de nudos a m/s (1 nudo = 0.514444 m/s)
            speed_mps = speed_knots[2] * 0.514444 if speed_knots else 0.0

        else:
             print("GPS Waiting for fix...")

    except Exception as e:
        print(f"Error GPS processing: {e}")

    return lat_decimal, lon_decimal, sats, speed_mps # Devolver velocidad en m/s


def datos():
    # Leer sensores con manejo básico de errores
    ax, ay, az, gx, gy, gz, temp_mpu = read_mpu_all() #safe_read(read_mpu_all, default_value=(0.0,) * 7)
    temp_bmp, presion = read_bmp_all() #safe_read(read_bmp_all, default_value=(0.0, 0.0))
    temp_dht, hum = read_dht_all()
    lat, lon, sats, speed = read_gps_all()

    # Devolver todos los datos en el orden esperado por la web
    return (ax, ay, az, gx, gy, gz, temp_mpu, temp_bmp, presion, temp_dht, hum, lat, lon, sats, speed)

def print_data():
    while True:
        sensor_values = datos()
        # Formatear la cadena: asegúrate de que todos sean números aquí
        # Usa formateo para controlar decimales si quieres
        output_line = ",".join(map(str, sensor_values)) + "\n"
        print(output_line, end='')
        #sys.stdout.flush()
        time.sleep(0.5) # Ajusta la frecuencia de envío (5 Hz en este ejemplo)
