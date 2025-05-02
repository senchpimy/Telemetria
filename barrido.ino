#include <Servo.h>

Servo myServo;  // Crear objeto para controlar el servo
int servoPin = 9; // Pin al que está conectado el servo

void setup() {
  myServo.attach(servoPin); // Conectar el servo al pin especificado
  myServo.write(0);         // Posición inicial 0°
  delay(1000);              // Esperar 1 segundo antes de iniciar el movimiento
}

void loop() {
  // Mover de 0° a 90° en 3 segundos
  for (int angle = 0; angle <= 90; angle++) {
    myServo.write(angle);
    delay(3000 / 90); // 3 segundos divididos en 90 pasos (~33ms por grado)
  }
  delay(1000); // Pausa de 1 segundo

  // Mover de 90° a 0° en 3 segundos
  for (int angle = 90; angle >= 0; angle--) {
    myServo.write(angle);
    delay(3000 / 90); // ~33ms por grado para 3 segundos totales
  }
  delay(1000); // Pausa antes de repetir
}
