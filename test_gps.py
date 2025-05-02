from machine import UART, Pin
from time import sleep
from micropyGPS import MicropyGPS

# Instantiate the micropyGPS object
my_gps = MicropyGPS()

# Define the UART pins and create a UART object
gps_serial = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))

while True:
    try:
        while gps_serial.any():
            data = gps_serial.read()
            for byte in data:
                stat = my_gps.update(chr(byte))
                if stat is not None:
                    # Print parsed GPS data
                    print('UTC Timestamp:', my_gps.timestamp)
                    print('Date:', my_gps.date_string('long'))
                    print('Latitude:', my_gps.latitude_string())
                    print('Longitude:', my_gps.longitude_string())
                    print('Altitude:', my_gps.altitude)
                    print('Satellites in use:', my_gps.satellites_in_use)
                    print(f"Velocidad (Knots): {my_gps.speed_string('knot')} knots")
                    # 1 nudo = 0.514444 metros por segundo
                    speed_knots = my_gps.speed[2] # speed es una tupla [Deg, Min, Knot]
                    speed_mps = speed_knots * 0.514444
                    print(f"Velocidad (m/s): {speed_mps:.2f} m/s")
                    print(f"Rumbo (grados): {my_gps.course:.1f}°")
                    print('Horizontal Dilution of Precision:', my_gps.hdop)
                    print()
            
    except Exception as e:
        print(f"An error occurred: {e}")
