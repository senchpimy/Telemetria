from machine import I2C, Pin
import time
import math
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)

class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
    def read_accel(self):
        data = self.i2c.readfrom_mem(self.addr, 0x3B, 6)
        ax = self._bytes_to_int(data[0:2])
        ay = self._bytes_to_int(data[2:4])
        az = self._bytes_to_int(data[4:6])
        return ax/16384, ay/16384, az/16384
    def read_gyro(self):
        data = self.i2c.readfrom_mem(self.addr, 0x43, 6)
        gx = self._bytes_to_int(data[0:2])
        gy = self._bytes_to_int(data[2:4])
        gz = self._bytes_to_int(data[4:6])
        return gx/131, gy/131, gz/131
    def _bytes_to_int(self, b):
        value = b[0] << 8 | b[1]
        if value > 32767:
            value -= 65536
        return value

class HMC5883L:
    def __init__(self, i2c, addr=0x1E):
        self.i2c = i2c
        self.addr = addr
        self.i2c.writeto_mem(self.addr, 0x00, b'\x70')
        self.i2c.writeto_mem(self.addr, 0x01, b'\xA0')
        self.i2c.writeto_mem(self.addr, 0x02, b'\x00')
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
        self.B1  = self._read_s16(0xB6)
        self.B2  = self._read_s16(0xB8)
        self.MB  = self._read_s16(0xBA)
        self.MC  = self._read_s16(0xBC)
        self.MD  = self._read_s16(0xBE)
    def _read_u16(self, reg):
        data = self.i2c.readfrom_mem(self.addr, reg, 2)
        return data[0] << 8 | data[1]
    def _read_s16(self, reg):
        result = self._read_u16(reg)
        if result > 32767:
            result -= 65536
        return result
    def read_raw_temp(self):
        self.i2c.writeto_mem(self.addr, 0xF4, b'\x2E')
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

def test_acelerometro_gravedad_estatica(sensor):
    print("Prueba de Acelerómetro: Verificación de Gravedad Estática")
    print("Coloca el sensor sobre una superficie plana, eje Z apuntando hacia arriba.")
    time.sleep(5)
    ax, ay, az = sensor.read_accel()
    print("Lecturas en reposo: X={:.2f}g, Y={:.2f}g, Z={:.2f}g".format(ax, ay, az))
    print("Gira el sensor 180° (Z hacia abajo) y observa los cambios...")
    time.sleep(5)
    ax, ay, az = sensor.read_accel()
    print("Lecturas invertidas: X={:.2f}g, Y={:.2f}g, Z={:.2f}g".format(ax, ay, az))

def test_inclinacion(sensor):
    print("Prueba de Acelerómetro: Inclinación Controlada")
    print("Inclina el sensor en los ejes X e Y a 30°, 45° y 90° respecto al suelo.")
    for _ in range(10):
        ax, ay, az = sensor.read_accel()
        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
        print("Roll: {:.2f}°, Pitch: {:.2f}°".format(roll, pitch))
        time.sleep(10)

def test_gyro_estabilidad(sensor):
    print("Prueba de Giroscopio: Estabilidad en Reposo (30 segundos)")
    sum_gx = sum_gy = sum_gz = 0
    count = 0
    start = time.time()
    while time.time() - start < 30:
        gx, gy, gz = sensor.read_gyro()
        sum_gx += gx
        sum_gy += gy
        sum_gz += gz
        count += 1
        time.sleep(0.1)
    bias_gx = sum_gx / count
    bias_gy = sum_gy / count
    bias_gz = sum_gz / count
    print("Bias del giroscopio: X={:.2f}°/s, Y={:.2f}°/s, Z={:.2f}°/s".format(bias_gx, bias_gy, bias_gz))

#def test_gyro_rotacion(sensor):
#    print("Prueba de Giroscopio: Rotación Controlada")
#    print("Gira el sensor 90° en aproximadamente 3 segundos.")
#    integrated_angle = 0
#    t0 = time.time()
#    prev_time = t0
#    duration = 3
#    while time.time() - t0 < duration:
#        current_time = time.time()
#        dt = current_time - prev_time
#        gx, gy, gz = sensor.read_gyro()
#        integrated_angle += gz * dt
#        print("Ángulo integrado: {:.2f}°".format(integrated_angle))
#        prev_time = current_time
#        time.sleep(0.1)
#    print("Ángulo final integrado: {:.2f}° (valor esperado ≈ 90°)".format(integrated_angle))

def test_gyro_rotacion(sensor):
    print("Prueba de Giroscopio: Rotación Controlada")
    print("Gira el sensor 90° en aproximadamente 3 segundos.")
    integrated_angle = 0
    t0 = time.time()
    prev_time = t0
    duration = 3
    while time.time() - t0 < duration:
        current_time = time.time()
        dt = current_time - prev_time
        gx, gy, gz = sensor.read_gyro()
        integrated_angle += gz * dt
        print("Velocidad angular (Z): {:.2f}°/s | Ángulo integrado: {:.2f}°".format(gz, integrated_angle))
        prev_time = current_time
        time.sleep(0.1)
    print("Ángulo final integrado: {:.2f}° (valor esperado ≈ 90°)".format(integrated_angle))

def test_magnetometro(mag):
    print("Prueba de Magnetómetro: Identificación del Norte Magnético")
    print("Gira el sensor y observa el ángulo (heading) calculado...")
    for _ in range(10):
        x, y, z = mag.read()
        heading = math.degrees(math.atan2(y, x))
        if heading < 0:
            heading += 360
        print("Heading: {:.2f}°".format(heading))
        time.sleep(1)

def test_bmp180_temperatura(bmp):
    print("Prueba de BMP180: Comparación de Temperatura")
    temp = bmp.read_temperature()
    print("Temperatura medida: {:.2f} °C".format(temp))
    print("Compara con un termómetro de referencia (error esperado ±2°C).")

def test_bmp180_presion(bmp):
    print("Prueba de BMP180: Presión con Diferencia de Altura")
    presuelo = bmp.read_pressure()
    print("Presión a nivel del suelo: {} Pa".format(presuelo))
    print("Sube el sensor a aproximadamente 1 metro de altura y presiona Enter...")
    input()
    pres_altura = bmp.read_pressure()
    print("Presión a 1 metro: {} Pa".format(pres_altura))
    diff = presuelo - pres_altura
    print("Diferencia de presión: {} Pa (valor esperado ≈ 12 Pa por metro)".format(diff))

#while (True):
#  mpu = MPU6050(i2c)
#  mag = HMC5883L(i2c)
#  bmp = BMP180(i2c)
#  print("Seleccione la prueba a ejecutar:")
#  print("1: Acelerómetro - Verificación de Gravedad Estática")
#  print("2: Acelerómetro - Inclinación Controlada")
#  print("3: Giroscopio - Estabilidad en Reposo")
#  print("4: Giroscopio - Rotación Controlada")
#  print("5: Magnetómetro - Identificación del Norte Magnético")
#  print("6: BMP180 - Comparación de Temperatura")
#  print("7: BMP180 - Prueba de Presión con Diferencia de Altura")
#  opcion = input("Ingrese el número de la prueba: ")
#  if opcion == "1":
#      test_acelerometro_gravedad_estatica(mpu)
#  elif opcion == "2":
#      test_inclinacion(mpu)
#  elif opcion == "3":
#      test_gyro_estabilidad(mpu)
#  elif opcion == "4":
#      test_gyro_rotacion(mpu)
#  elif opcion == "5":
#      test_magnetometro(mag)
#  elif opcion == "6":
#      test_bmp180_temperatura(bmp)
#  elif opcion == "7":
#      test_bmp180_presion(bmp)
#  else:
#      print("Opción no válida.")

#from machine import I2C, Pin
#import time
#import math
#
## Inicializar el bus I2C solo una vez
#i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
## Realizar un escaneo I2C para verificar dispositivos conectados
#print("Dispositivos I2C encontrados:", i2c.scan())
#
## Inicializar los sensores una vez, fuera del bucle
mpu = MPU6050(i2c)
mag = HMC5883L(i2c)
bmp = BMP180(i2c)

while True:
    print("Seleccione la prueba a ejecutar:")
    print("1: Acelerómetro - Verificación de Gravedad Estática")
    print("2: Acelerómetro - Inclinación Controlada")
    print("3: Giroscopio - Estabilidad en Reposo")
    print("4: Giroscopio - Rotación Controlada")
    print("5: Magnetómetro - Identificación del Norte Magnético")
    print("6: BMP180 - Comparación de Temperatura")
    print("7: BMP180 - Prueba de Presión con Diferencia de Altura")
    opcion = input("Ingrese el número de la prueba: ")
    if opcion == "1":
        test_acelerometro_gravedad_estatica(mpu)
    elif opcion == "2":
        test_inclinacion(mpu)
    elif opcion == "3":
        test_gyro_estabilidad(mpu)
    elif opcion == "4":
        test_gyro_rotacion(mpu)
    elif opcion == "5":
        test_magnetometro(mag)
    elif opcion == "6":
        test_bmp180_temperatura(bmp)
    elif opcion == "7":
        test_bmp180_presion(bmp)
    else:
        print("Opción no válida.")
