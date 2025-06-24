import ustruct
import utime
from machine import Pin, SPI
from nrf24l01 import NRF24L01  # Asegúrate de tener este archivo en el Pico
import nrf24l01 as nr

ce_pin = 17  # Cambair este en el emisor
csn_pin = 21
spi_sck_pin = 18
spi_tx_pin = 19  # MOSI
spi_rx_pin = 16  # MISO

pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xd2\xf0\xf0\xf0\xf0")


# --- Inicialización ---
def setup_test(canal=15, poder=nr.POWER_0, velocidad=nr.SPEED_250K):
    print("Inicializando NRF24L01 como transmisor...")
    # Configura SPI
    spi = SPI(0, sck=Pin(spi_sck_pin), mosi=Pin(spi_tx_pin), miso=Pin(spi_rx_pin))
    # Crea el objeto NRF24L01
    nrf = NRF24L01(
        spi, Pin(csn_pin, Pin.OUT), Pin(ce_pin, Pin.OUT), payload_size=32
    )  # payload_size debe coincidir
    # Configurar para la prueba de loopback de ACK
    nrf.open_tx_pipe(pipes[1])  # Dirección a la que se envía
    nrf.open_rx_pipe(1, pipes[0])
    # Pipe para recibir ACKs (AutoACK usa Pipe 0 implicitamente por la dirección TX)
    # Abrir Pipe 1 aquí con la dirección base puede ser redundante pero no daña
    # La librería debería manejar la escucha del ACK en la dirección correcta (pipe[0])
    nrf.stop_listening()  # Poner en modo listo para transmitir
    # La librería habilita AutoACK por defecto en los pipes abiertos
    nrf.set_channel(canal)  # Elige un canal (0-125)
    nrf.set_power_speed(poder, velocidad)  # Max potencia, min velocidad para robustez
    nrf.stop_listening()  # Empezar en modo transmisor (o listo para transmitir)
    utime.sleep_ms(20)
    print("Configuración NRF24L01 completa.")
    print(f"  TX Address: {pipes[0]}")
    print(f"  RX Pipe 1 Address: {pipes[1]} (Abierto con dirección TX para ACK)")
    # print(f"  Channel: {nrf.get_channel()}")
    return nrf


PAYLOAD_FORMAT = "<h6hhihBffB"
EXPECTED_PAYLOAD_SIZE = ustruct.calcsize(PAYLOAD_FORMAT)  # Debería ser 32
SCALE_FACTOR_ACCEL = 1000.0
SCALE_FACTOR_GYRO = 100.0
SCALE_FACTOR_TEMP = 100.0


def enviar_nrf(
    funcion_datos, canal=15, poder=nr.POWER_0, velocidad=nr.SPEED_250K, espera=500
):
    nrf = setup_test(canal, poder, velocidad)  # Inicializa el NRF24L01
    # Asumiendo que nrf está inicializado correctamente con payload_size=32
    # desde una función setup() como la tenías antes.
    # nrf = setup() # Asegúrate que setup() configura payload_size=EXPECTED_PAYLOAD_SIZE
    fails = 0
    max_fails = 50
    counter = 0  # Inicializa el contador de paquetes
    while fails < max_fails:
        print(f"\n--- Preparando paquete {counter} ---")
        # 1. Recolectar datos
        try:
            (
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
            ) = funcion_datos()
        except Exception as e:
            print(f"Error Crítico al obtener datos: {e}. Reintentando...")
            utime.sleep_ms(1000)
            continue  # Saltar este intento de envío
        # 2. Escalar y preparar para empaquetar
        # Multiplicar por factor y convertir a entero para 'h' y 'B'
        ax_h = int(ax * SCALE_FACTOR_ACCEL)
        ay_h = int(ay * SCALE_FACTOR_ACCEL)
        az_h = int(az * SCALE_FACTOR_ACCEL)
        gx_h = int(gx * SCALE_FACTOR_GYRO)
        gy_h = int(gy * SCALE_FACTOR_GYRO)
        gz_h = int(gz * SCALE_FACTOR_GYRO)
        bmp_t_h = int(temp_bmp * SCALE_FACTOR_TEMP)
        pres_i = int(presion)  # La presión va como entero directamente
        dht_t_h = int(temp_dht * SCALE_FACTOR_TEMP)  # Usamos temp en C para DHT
        hum_b = int(hum)
        sats_b = int(sats)
        # lat, lon ya son floats (f)
        # counter ya es un entero (lo usaremos con 'h')
        # 3. Empaquetar los datos
        try:
            payload = ustruct.pack(
                PAYLOAD_FORMAT,
                counter,  # h
                ax_h,
                ay_h,
                az_h,  # 3 * h
                gx_h,
                gy_h,
                gz_h,  # 3 * h
                bmp_t_h,  # h
                pres_i,  # i
                dht_t_h,  # h
                hum_b,  # B
                0,  # lat,           # f
                0,  # lon,           # f
                sats_b,  # B
            )
            print(f"Payload Empaquetado ({len(payload)} bytes): {payload}")
        except Exception as e:
            print(f"Error empaquetando datos: {e}. Reintentando...")
            counter += (
                1  # Incrementar incluso si falla el empaquetado para no estancarse
            )
            utime.sleep_ms(1000)
            continue
        # 4. Enviar el paquete
        print(f"Enviando paquete {counter} ... ", end="")
        try:
            nrf.send(payload)
            nrf.send(payload)  # Duplicar el envío para asegurar recepción
            print("Éxito (ACK Recibido)")
            counter += 1  # Incrementar contador SOLO si el envío fue exitoso
            fails = 0
        except OSError:
            print("FALLO (Timeout / Sin ACK)")
            fails += 1
            # No incrementamos counter aquí, se reintentará el mismo paquete
        except Exception as e:
            print(f"Error inesperado en nrf.send: {e}")
            fails += 1  # Considerar como fallo
        utime.sleep_ms(espera)  # Esperar antes del siguiente paquete
    print(f"\n--- {max_fails} fallos consecutivos, deteniendo transmisión ---")
    # ... (código de apagado opcional) ...
