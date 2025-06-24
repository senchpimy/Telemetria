import ustruct
import utime
from machine import Pin, SPI
from nrf24l01 import NRF24L01  # Asegúrate de tener este archivo en el Pico
import nrf24l01 as nr

# --- Configuración de Pines --- (Igual que el transmisor)
ce_pin = 22
csn_pin = 21
spi_sck_pin = 18
spi_tx_pin = 19  # MOSI
spi_rx_pin = 16  # MISO

# --- Configuración del nRF24L01 ---
# Las direcciones deben coincidir con las del transmisor, pero intercambiadas
# El receptor escucha en la dirección a la que el transmisor envía (pipes[0])
pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xd2\xf0\xf0\xf0\xf0")


# --- Inicialización ---
def setup_test(canal=15,poder=nr.POWER_0, velocidad=nr.SPEED_250K):
    print("Inicializando NRF24L01 como receptor...")
    # Configura SPI
    spi = SPI(0, sck=Pin(spi_sck_pin), mosi=Pin(spi_tx_pin), miso=Pin(spi_rx_pin))
    # Crea el objeto NRF24L01
    nrf = NRF24L01(
        spi, Pin(csn_pin, Pin.OUT), Pin(ce_pin, Pin.OUT), payload_size=32
    )  # payload_size debe coincidir
    # Configurar para la prueba de loopback de ACK
    # La librería habilita AutoACK por defecto en los pipes abiertos
    nrf.open_tx_pipe(pipes[0])  # Dirección base del transmisor (para enviar ACKs)
    nrf.open_rx_pipe(1, pipes[1])
    # Pipe 1 para escuchar los datos entrantes (¡IMPORTANTE!)
    nrf.set_channel(canal)  # Elige un canal (0-125)
    nrf.set_power_speed(
        poder, velocidad
    )  # Max potencia, min velocidad para robustez
    nrf.start_listening()
    utime.sleep_ms(20)
    print("Configuración NRF24L01 completa.")
    print(f"  TX Address: {pipes[0]}")
    print(f"  RX Pipe 1 Address: {pipes[1]} (Abierto con dirección TX para ACK)")
    # print(f"  Channel: {nrf.get_channel()}")
    return nrf




SCALE_FACTOR_ACCEL = 1000.0
SCALE_FACTOR_GYRO = 100.0
SCALE_FACTOR_TEMP = 100.0

# Define el formato del struct para el payload EXACTAMENTE igual en sender y receiver
PAYLOAD_FORMAT = '<h6hhihBffB'
EXPECTED_PAYLOAD_SIZE = ustruct.calcsize(PAYLOAD_FORMAT) # Debería ser 32

def receptor(canal=15,poder=nr.POWER_0, velocidad=nr.SPEED_250K):
    nrf = setup_test(canal,poder,velocidad)  # Inicializa el NRF24L01
    # Asumiendo que nrf está inicializado correctamente con payload_size=32
    # y puesto en modo escucha (start_listening()) fuera de esta función.
    print(f"\n--- Esperando paquetes (Formato: {PAYLOAD_FORMAT}, Tamaño: {EXPECTED_PAYLOAD_SIZE} bytes) ---")
    while True:
        try:
            if nrf.any():
                while nrf.any():
                    payload = nrf.recv()
                    print(f"\nRaw payload received ({len(payload)} bytes): {payload}")
                    # Verificar tamaño antes de desempaquetar
                    if len(payload) == EXPECTED_PAYLOAD_SIZE:
                        try:
                            # Desempaquetar usando el formato definido
                            unpacked_data = ustruct.unpack(PAYLOAD_FORMAT, payload)
                            # Asignar a variables legibles en el orden correcto
                            (counter,
                             ax_h, ay_h, az_h,
                             gx_h, gy_h, gz_h,
                             bmp_t_h,
                             pres_i,
                             dht_t_h,
                             hum_b,
                             lat, lon,
                             sats_b) = unpacked_data
                            # Des-escalar los valores
                            ax = ax_h / SCALE_FACTOR_ACCEL
                            ay = ay_h / SCALE_FACTOR_ACCEL
                            az = az_h / SCALE_FACTOR_ACCEL
                            gx = gx_h / SCALE_FACTOR_GYRO
                            gy = gy_h / SCALE_FACTOR_GYRO
                            gz = gz_h / SCALE_FACTOR_GYRO
                            bmp_t = bmp_t_h / SCALE_FACTOR_TEMP
                            # pres_i ya está en Pascales
                            dht_t = dht_t_h / SCALE_FACTOR_TEMP # Temp DHT en C
                            #Crear string para mandar como csv
                            output_line = f"{counter},{ax},{ay},{az},{gx},{gy},{gz},{bmp_t},{pres_i},{dht_t},{hum_b},{lat},{lon},{sats_b}\n"

                            # hum_b ya está en %
                            # lat, lon ya son floats
                            # sats_b ya es el número de satélites
                            # Imprimir los datos decodificados
                            print(f"--- Paquete Decodificado #{counter} ---")
                            print(f"  MPU Accel: X={ax:+.2f}g Y={ay:+.2f}g Z={az:+.2f}g")
                            print(f"  MPU Gyro:  X={gx:+.1f}dps Y={gy:+.1f}dps Z={gz:+.1f}dps")
                            print(f"  BMP Temp:  {bmp_t:.2f} C")
                            print(f"  BMP Press: {pres_i} Pa")
                            print(f"  DHT Temp:  {dht_t:.1f} C")
                            print(f"  DHT Hum:   {hum_b:.1f} %")
                            print(f"  GPS Pos:   Lat={lat:.6f}, Lon={lon:.6f}")
                            print(f"  GPS Sats:  {sats_b}")
                            print("-" * 30)
                            print(output_line)
                        except Exception as e:
                            print(f"Error desempaquetando payload: {e}")
                            # Podrías imprimir el payload crudo aquí para depurar
                            print(f"Payload problemático: {payload}")
                    else:
                        print(f"Error: Payload recibido con tamaño incorrecto ({len(payload)} bytes), esperado {EXPECTED_PAYLOAD_SIZE}.")
                        # Es útil ver el payload incorrecto
                        print(f"Payload incorrecto: {payload}")
                # Pequeña pausa después de vaciar el buffer FIFO si se recibieron datos
                utime.sleep_ms(10)
            # Pequeña pausa general para no consumir 100% CPU
            utime.sleep_ms(5) # Ajusta según sea necesario
        except KeyboardInterrupt:
            print("\nDetenido por el usuario.")
            break
        except Exception as e:
            print(f"Error en el bucle principal del receptor: {e}")
            utime.sleep_ms(1000) # Pausa más larga en caso de error grave
