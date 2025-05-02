from machine import Pin, PWM
from time import sleep

# Configurar el pin GPIO28_A2 para PWM
servo = PWM(Pin(28))
servo.freq(50)  # Frecuencia típica para servos

# Función para establecer el ángulo del servo
def set_angle(angle):
    # Calcular el ciclo de trabajo (duty) correspondiente al ángulo
    duty = int((angle / 180) * 5000 + 2500)  # Ajuste típico para servos
    servo.duty_u16(duty)

# Mover de 0° a 90° en 3 segundos
start_angle = 0
end_angle = 90
steps = 30
delay = 3 / steps  # 3 segundos divididos por la cantidad de pasos

for angle in range(start_angle, end_angle + 1, int((end_angle - start_angle) / steps)):
    set_angle(angle)
    sleep(delay)

print("Movimiento completado.")
