# Telemetria Project

## Descripción
Este proyecto está enfocado en la telemetría, abarcando la comunicación, adquisición de datos de sensores y pruebas de ignición, diseñado específicamente para **Raspberry Pi Pico** utilizando **MicroPython**.

## Estructura del Proyecto
- **Cominucacion/**: Contiene scripts y archivos relacionados con la comunicación de datos.
- **lib/**: Librerías para diferentes sensores y módulos (ADXL345, BMP180, MPU6050, NRF24L01, GPS, etc.).
- **Tests/**: Pruebas específicas, incluyendo las de ignición.

## Instalación y Configuración (Raspberry Pi Pico con MicroPython)
Para configurar su Raspberry Pi Pico y el entorno de desarrollo, siga los siguientes pasos:

1.  **Instalar Firmware MicroPython en Raspberry Pi Pico:**
    *   Descargue el firmware UF2 de MicroPython para Raspberry Pi Pico desde el sitio oficial de MicroPython.
    *   Conecte su Raspberry Pi Pico a su computadora mientras mantiene presionado el botón `BOOTSEL`. Debería aparecer como un dispositivo de almacenamiento masivo (RPI-RP2).
    *   Arrastre y suelte el archivo UF2 descargado en el dispositivo RPI-RP2. El Pico se reiniciará con MicroPython.

2.  **Transferir Archivos al Raspberry Pi Pico:**
    Utilice una herramienta como `rshell`, `ampy`, o el IDE Thonny para transferir los archivos del proyecto (especialmente los de `lib/` y `Cominucacion/`) a su Raspberry Pi Pico.

    **Ejemplo con Thonny:**
    *   Abra Thonny y configure el intérprete para "MicroPython (Raspberry Pi Pico)".
    *   Conecte su Pico.
    *   Puede arrastrar y soltar archivos directamente al panel "Files" de Thonny, o usar "File > Save as..." para guardar archivos en el dispositivo MicroPython.

    Asegúrese de que las librerías en `lib/` estén en el directorio raíz del Pico o en un directorio accesible por MicroPython (e.g., `/lib` en el Pico si su código las busca allí).

## Conexiones y Pines (Raspberry Pi Pico)
El Raspberry Pi Pico ofrece una variedad de pines GPIO que pueden ser configurados para diferentes propósitos (I2C, SPI, UART, Digital I/O, ADC). A continuación, se presentan algunas configuraciones comunes y consideraciones para los sensores utilizados en este proyecto.

**Consideraciones Generales:**
*   **Alimentación:** La mayoría de los sensores requerirán 3.3V (pin 36 o 3V3_OUT) y GND (varios pines, e.g., pin 38).
*   **Nivel Lógico:** El Raspberry Pi Pico opera a 3.3V. Asegúrese de que sus sensores sean compatibles con este nivel lógico o utilice un conversor de nivel si es necesario.

**Interfaces Comunes:**

### I2C (Inter-Integrated Circuit)
Utilizado por sensores como el ADXL345, BMP180, MPU6050.
El Raspberry Pi Pico tiene dos controladores I2C (I2C0 e I2C1).

*   **I2C0 (Default en MicroPython):**
    *   SDA: GP0 (pin 1)
    *   SCL: GP1 (pin 2)
    *   Alternativas: GP4/GP5, GP8/GP9, GP12/GP13, GP16/GP17, GP20/GP21

*   **I2C1:**
    *   SDA: GP2 (pin 4)
    *   SCL: GP3 (pin 5)
    *   Alternativas: GP6/GP7, GP10/GP11, GP14/GP15, GP18/GP19, GP26/GP27

**Ejemplo de Conexión I2C (MPU6050):**
*   MPU6050 VCC -> Pico 3.3V (pin 36)
*   MPU6050 GND -> Pico GND (pin 38)
*   MPU6050 SDA -> Pico GP0 (pin 1)
*   MPU6050 SCL -> Pico GP1 (pin 2)

### SPI (Serial Peripheral Interface)
Utilizado por módulos como el NRF24L01.
El Raspberry Pi Pico tiene dos controladores SPI (SPI0 y SPI1).

*   **SPI0 (Default en MicroPython):**
    *   SCK: GP2 (pin 4)
    *   MOSI: GP3 (pin 5)
    *   MISO: GP4 (pin 6)
    *   Alternativas: GP6/GP7/GP8, GP18/GP19/GP16

*   **SPI1:**
    *   SCK: GP10 (pin 14)
    *   MOSI: GP11 (pin 15)
    *   MISO: GP12 (pin 16)
    *   Alternativas: GP14/GP15/GP13

**Ejemplo de Conexión SPI (NRF24L01):**
*   NRF24L01 VCC -> Pico 3.3V (pin 36)
*   NRF24L01 GND -> Pico GND (pin 38)
*   NRF24L01 SCK -> Pico GP2 (pin 4)
*   NRF24L01 MOSI -> Pico GP3 (pin 5)
*   NRF24L01 MISO -> Pico GP4 (pin 6)
*   NRF24L01 CE -> Pico GPX (GPIO configurable, e.g., GP17)
*   NRF24L01 CSN -> Pico GPY (GPIO configurable, e.g., GP16)

### UART (Universal Asynchronous Receiver/Transmitter)
Utilizado por módulos GPS.
El Raspberry Pi Pico tiene dos controladores UART (UART0 y UART1).

*   **UART0 (Default en MicroPython):**
    *   TX: GP0 (pin 1)
    *   RX: GP1 (pin 2)
    *   Alternativas: GP12/GP13, GP16/GP17

*   **UART1:**
    *   TX: GP4 (pin 6)
    *   RX: GP5 (pin 7)
    *   Alternativas: GP8/GP9, GP10/GP11, GP14/GP15

**Ejemplo de Conexión UART (Módulo GPS):**
*   GPS VCC -> Pico 3.3V (pin 36)
*   GPS GND -> Pico GND (pin 38)
*   GPS TX -> Pico GP1 (pin 2) (RX de Pico)
*   GPS RX -> Pico GP0 (pin 1) (TX de Pico)

**Nota Importante:** Los pines GPIO específicos para cada sensor (CE, CSN para NRF24L01, o interrupciones) pueden ser configurados en el código MicroPython. Consulte la documentación de cada librería en `lib/` y los datasheets de los sensores para configuraciones detalladas.

## Uso
Aquí se describirá cómo ejecutar las diferentes partes del proyecto en su Raspberry Pi Pico.

### Comunicación
Para ejecutar los scripts de comunicación en su Pico, transfiera los archivos a su dispositivo y ejecútelos desde la terminal de MicroPython (e.g., a través de Thonny o rshell).
```python
# Ejemplo de ejecución en la terminal de MicroPython:
# import Cominucacion.transmicion
# import Cominucacion.receptor
```

### Sensores
Las librerías en `lib/` están diseñadas para ser importadas y utilizadas en sus propios scripts de MicroPython en el Pico.
```python
# Ejemplo de uso de un sensor (e.g., MPU6050) en un script en el Pico
from machine import Pin, I2C
from lib.MPU6050 import MPU6050

# Configurar I2C0
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
mpu = MPU6050(i2c)

# Leer datos
accel = mpu.read_accel_data()
print("Aceleración:", accel)
```

### Pruebas de Ignición
Para ejecutar las pruebas de ignición, navegue al directorio `Tests/ignicion` y siga las instrucciones específicas de ese módulo. Es probable que necesite un entorno de desarrollo como PlatformIO configurado para Raspberry Pi Pico.
```bash
# Ejemplo de comandos para PlatformIO (ejecutar en su computadora, no en el Pico):
# platformio run -t upload # Para subir el firmware de prueba
# platformio device monitor # Para ver la salida de la prueba
```

## Licencia
Este proyecto está bajo la Licencia MIT. Consulte el archivo `LICENSE` para más detalles.
