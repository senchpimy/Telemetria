import time


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
        self.B1 = self._read_s16(0xB6)
        self.B2 = self._read_s16(0xB8)
        self.MB = self._read_s16(0xBA)
        self.MC = self._read_s16(0xBC)
        self.MD = self._read_s16(0xBE)

    def _read_u16(self, reg):
        data = self.i2c.readfrom_mem(self.addr, reg, 2)
        return data[0] << 8 | data[1]

    def _read_s16(self, reg):
        result = self._read_u16(reg)
        if result > 32767:
            result -= 65536
        return result

    def read_raw_temp(self):
        self.i2c.writeto_mem(self.addr, 0xF4, b"\x2e")
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
