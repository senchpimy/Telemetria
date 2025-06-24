from machine import I2C, Pin, UART
import time
import math
import dht
import adxl345
from MPU6050 import MPU6050
from micropyGPS import MicropyGPS

my_gps = MicropyGPS()

gps_serial = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))

sensor = dht.DHT11(Pin(22))

i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)


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


# ----------------------Transmicion---------------------
import ustruct
import utime
from machine import Pin, SPI
from nrf24l01 import NRF24L01  # Asegúrate de tener este archivo en el Pico
import nrf24l01 as nr
from BMP180 import BMP180  # Asegúrate de tener la librería BMP180 correcta
import bmp280

mpu = MPU6050(i2c)
# mag = HMC5883L(i2c)
bmp = BMP180(i2c)
bmp2 = bmp280.BMP280(i2c)

# -----------------------Recolectar datos-----------------------
# Asumiendo que NRF24L01, machine, y las clases de sensores están importadas
# Asumiendo que nrf, mpu, bmp, sensor, gps_serial, my_gps están inicializados globalmente o pasados

# --- Constantes de Escala ---
# Usar el mismo factor al empaquetar y desempaquetar
SCALE_FACTOR_ACCEL = 1000.0
SCALE_FACTOR_GYRO = 100.0
SCALE_FACTOR_TEMP = 100.0

# Define el formato del struct para el payload EXACTAMENTE igual en sender y receiver
PAYLOAD_FORMAT = "<h6hhihBffB"
EXPECTED_PAYLOAD_SIZE = ustruct.calcsize(PAYLOAD_FORMAT)  # Debería ser 32

# Asegúrate de inicializar el NRF con el tamaño correcto en tu setup()
# nrf = NRF24L01(spi, csn, ce, payload_size=EXPECTED_PAYLOAD_SIZE) # payload_size=32


def safe_read(
    sensor_func, default_value=(0.0,) * 7
):  # Ajusta el número de valores por defecto
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
    temp_bmp = bmp.read_temperature()  # O bmp.read_temperature()
    presion = bmp.read_pressure()  # O bmp.read_pressure()
    return temp_bmp, presion


def read_dht_all():
    try:
        sensor.measure()
        temp_dht = sensor.temperature()
        hum = sensor.humidity()
        return temp_dht, hum
    except Exception as e:
        print(f"Error DHT: {e}")
        return float("nan"), float("nan")  # Valores por defecto


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
            lat_decimal = (
                my_gps.latitude_decimal if hasattr(my_gps, "latitude_decimal") else 0.0
            )
            lon_decimal = (
                my_gps.longitude_decimal
                if hasattr(my_gps, "longitude_decimal")
                else 0.0
            )
            sats = my_gps.satellites_in_use
            speed_knots = (
                my_gps.speed_knots() if hasattr(my_gps, "speed_knots") else [0.0]
            )  # devuelve lista speed[2] = knots

            # Convierte velocidad de nudos a m/s (1 nudo = 0.514444 m/s)
            speed_mps = speed_knots[2] * 0.514444 if speed_knots else 0.0

        else:
            print("GPS Waiting for fix...")

    except Exception as e:
        print(f"Error GPS processing: {e}")

    return lat_decimal, lon_decimal, sats, speed_mps  # Devolver velocidad en m/s


import adxl345

adxl = adxl345.ADXL345(i2c)  # Inicializar el acelerómetro ADXL345


def read_adxl345():
    try:
        adxl.start_measurements()
        time.sleep(0.1)  # Esperar un poco para que el sensor se estabilice
        ax, ay, az = adxl.get_readings()  # Leer los valores de aceleración
        adxl.stop_measurements()
        return ax, ay, az
    except Exception as e:
        print(f"Error reading ADXL345: {e}")
        return 0.0, 0.0, 0.0  # Valores por defecto en caso de error


def read_bmp280():
    try:
        temp_bmp280 = bmp2.temperature
        presion_bmp280 = bmp2.pressure
        return temp_bmp280, presion_bmp280
    except Exception as e:
        print(f"Error reading BMP280: {e}")
        return float("nan"), float("nan")  # Valores por defecto en caso de error


def datos():
    # Leer sensores con manejo básico de errores
    ax, ay, az, gx, gy, gz, temp_mpu = (
        read_mpu_all()
    )  # safe_read(read_mpu_all, default_value=(0.0,) * 7)
    temp_bmp, presion = (
        read_bmp_all()
    )  # safe_read(read_bmp_all, default_value=(0.0, 0.0))
    temp_dht, hum = read_dht_all()
    lat, lon, sats, speed = read_gps_all()

    # Leer ADXL345
    ax_adxl, ay_adxl, az_adxl = read_adxl345()

    # Leer BMP280
    temp_bmp280, presion_bmp280 = read_bmp280()

    # Devolver todos los datos en el orden esperado por la web
    return (
        ax,
        ay,
        az,
        gx,
        gy,
        gz,
        temp_mpu,
        temp_bmp,
        presion,
        temp_dht,
        hum,
        lat,
        lon,
        sats,
        speed,
        ax_adxl,
        ay_adxl,
        az_adxl,
        temp_bmp280,
        presion_bmp280,
    )


def print_data():
    while True:
        sensor_values = datos()
        print(
            f"MPU: Ax={sensor_values[0]}, Ay={sensor_values[1]}, Az={sensor_values[2]}, "
            f"Gx={sensor_values[3]}, Gy={sensor_values[4]}, Gz={sensor_values[5]}, "
            f"Temp MPU={sensor_values[6]} C"
        )
        print(f"BMP180: Temp={sensor_values[7]} C, Pressure={sensor_values[8]} Pa")
        print(f"DHT: Temp={sensor_values[9]} C, Humidity={sensor_values[10]} %")
        print(
            f"GPS: Lat={sensor_values[11]}, Lon={sensor_values[12]}, "
            f"Sats={sensor_values[13]}, Speed={sensor_values[14]} m/s"
        )
        print(
            f"ADXL345: Ax={sensor_values[15]}, Ay={sensor_values[16]}, Az={sensor_values[17]}"
        )
        print(f"BMP280: Temp={sensor_values[18]} C, Pressure={sensor_values[19]} Pa")
        print("-" * 50)
