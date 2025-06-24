import time
import struct
import machine
from micropython import const

DEVICE_ID_REGISTER_ADDR = const(0x00)
DEVICE_ADDRESS = const(0x53)
DEVICE_ID = const(0xE5)
POWER_CONTROL_REGISTER_ADDR = const(0x2D)
DATA_REGISTER_ADDR = const(0x32)
DATA_FORMAT_REGISTER_ADDR = const(0x31)

ACTIVE_MODE = const(0x28)
STANDBY_MODE = const(0x24)

EARTH_GRAVITY = const(9.80665)
SCALE_MULTIPLIER = const(0.004)

RANGE_2G = const(0)
RANGE_4G = const(1)
RANGE_8G = const(2)
RANGE_16G = const(3)


class DataFormatSettings:
    def __init__(self, initial_data: bytes):
        data = self.__parse_bits(initial_data)

        self.self_test = data[0]
        self.spi = data[1]
        self.int_invert = data[2]
        self.d4 = data[3]
        self.full_res = data[4]
        self.justify = data[5]
        self.range_d0 = data[6]
        self.range_d1 = data[7]

    def __get_combined_values(self) -> list[int]:
        return [
            self.self_test,
            self.spi,
            self.int_invert,
            self.d4,
            self.full_res,
            self.justify,
            self.range_d0,
            self.range_d1,
        ]

    def __parse_bits(self, data: bytes) -> list[int]:
        # return f"{data[0]:0>8b}"
        bits = []

        for ind in range(8):
            bits.append((data[0] >> 7 - ind) & 1)

        return bits

    def __combine_settings(self) -> str:
        return "".join([str(item) for item in self.__get_combined_values()])

    def dump(self) -> bytearray:
        seq = self.__combine_settings()
        return bytearray([int(seq, 2)])

    def set_range(self, range: int):
        self.full_res = 1
        self.range_d0 = (range >> 1) & 1
        self.range_d1 = (range >> 0) & 1

    def __str__(self):
        return self.__combine_settings()


class ADXL345:
    def __init__(self, i2c: machine.I2C, address=DEVICE_ADDRESS, range=RANGE_2G):
        self.i2c = i2c
        self.address = address

        self.range = range
        self.__final_readings_factor = EARTH_GRAVITY * SCALE_MULTIPLIER

        self.__set_measurements_range()

    def is_device_accessible(self) -> bool:
        return True if self.address in self.i2c.scan() else False

    def check_connection(self) -> bool:
        data = self.i2c.readfrom_mem(self.address, DEVICE_ID_REGISTER_ADDR, 1)

        return True if data == bytearray((DEVICE_ID,)) else False

    def __write_to_mem(self, reg_address: int, data: bytearray, delay=50):
        self.i2c.writeto_mem(self.address, reg_address, data)
        time.sleep_ms(delay)

    def __get_data_format_settings(self):
        initial_settings = self.i2c.readfrom_mem(
            self.address, DATA_FORMAT_REGISTER_ADDR, 1
        )
        return DataFormatSettings(initial_settings)

    def __set_measurements_range(self):
        settings = self.__get_data_format_settings()
        settings.set_range(self.range)

        self.__write_to_mem(DATA_FORMAT_REGISTER_ADDR, settings.dump())

    def start_measurements(self):
        self.__write_to_mem(POWER_CONTROL_REGISTER_ADDR, bytearray([ACTIVE_MODE]))

    def stop_measurements(self):
        self.__write_to_mem(POWER_CONTROL_REGISTER_ADDR, bytearray([STANDBY_MODE]), 0)

    def get_readings(self) -> tuple[float, float, float]:
        """returns acceleration for x, y, z axis"""

        data = self.i2c.readfrom_mem(self.address, DATA_REGISTER_ADDR, 6)
        raw_data_struct = struct.unpack("<hhh", data)
        processed_data = [val * self.__final_readings_factor for val in raw_data_struct]

        return processed_data[0], processed_data[1], processed_data[2]
