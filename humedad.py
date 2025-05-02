from machine import Pin
from time import sleep
import dht

sensor = dht.DHT11(Pin(22))
while True:
    try:
        sleep(2)
        sensor.measure()
        temp = sensor.temperature()
        hum = sensor.humidity()
        temp_f = temp * (9 / 5) + 32.0
        print("Temperature: %3.1f C" % temp)
        print("Temperature: %3.1f F" % temp_f)
        print("Humidity: %3.1f %%\n" % hum)
        ax, ay, az = mpu.read_accel()
        gx, gy, gz = mpu.read_gyro()
        print(
            f"MPU6050 - Accel: X={ax:+.2f}g Y={ay:+.2f}g Z={az:+.2f}g | Gyro: X={gx:+.1f} Y={gy:+.1f} Z={gz:+.1f} dps"
        )
        temperatura = bmp.read_temperature()
        presion = bmp.read_pressure()  # Usará el B5 calculado por read_temperature
        print(f"BMP180   - Temp: {temperatura:.2f} C | Press: {presion:.2f} Pa")
    except OSError as e:
        print("Failed to read sensor.")
