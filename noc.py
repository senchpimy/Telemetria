from machine import ADC, Pin, I2C
import dht
import time
from imu import MPU6050

# Objeto para acceder al pin analógico del sensor de temperatura LM35DZ
tempadc = ADC(0)

# Objeto para acceder al sensor de temperatura y humedad DHT11
dht_sensor = dht.DHT11(Pin(22))

# Configuración del LED para indicar que la Raspberry Pi Pico está encendida
LED = Pin("LED", Pin.OUT)
LED.on()

# Configuración del sensor MPU6050 para leer acelerómetro y giroscopio
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c)


def lm35_read_temperature():
    """Lee el sensor de temperatura LM35DZ y devuelve un promedio de 10 lecturas."""
    temp = 0
    for _ in range(10):
        temp += tempadc.read_u16() * 330 / 65536
        time.sleep_us(100)
    return temp / 10


while True:
    # Leer temperatura del sensor LM35DZ
    # lm35_temperature = lm35_read_temperature()
    # print(f"Temperatura leída por LM35DZ: {lm35_temperature}")
    # Leer temperatura y humedad del sensor DHT11
    dht_sensor.measure()
    dht_temperature = dht_sensor.temperature()
    humidity = dht_sensor.humidity()
    print(f"Temperatura DHT11: {dht_temperature}")
    print(f"Humedad: {humidity}")
    time.sleep(1)

    # Leer datos del sensor MPU6050
    ax = round(imu.accel.x, 2)
    ay = round(imu.accel.y, 2)
    az = round(imu.accel.z, 2)
    gx = round(imu.gyro.x)
    gy = round(imu.gyro.y)
    gz = round(imu.gyro.z)
    tem = round(imu.temperature, 2)
    print(
        f"ax: {ax}\tay: {ay}\taz: {az}\tgx: {gx}\tgy: {gy}\tgz: {gz}\tTemperature: {tem}",
        end="\r",
    )

    # Esperar un segundo entre lecturas
    time.sleep(1)
