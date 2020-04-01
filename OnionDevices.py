#class for devices
from OmegaExpansion import onionI2C
import time

class BME280(object):
    """docstring for BME280."""

    def __init__(self, i2c):
        super(BME280, self).__init__()
        self.I2C = i2c
        self.dig_T1 = 0
        self.dig_T2 = 0
        self.dig_T3 = 0
        self.dig_H1 = 0
        self.dig_H2 = 0
        self.dig_H3 = 0
        self.dig_H4 = 0
        self.dig_H5 = 0
        self.dig_H6 = 0
        self.dig_H7 = 0
        self.dig_P1 = 0
        self.dig_P2 = 0
        self.dig_P3 = 0
        self.dig_P4 = 0
        self.dig_P5 = 0
        self.dig_P6 = 0
        self.dig_P7 = 0
        self.dig_P8 = 0
        self.dig_P9 = 0

    def initialize(self):
        """
        Get intial Values from BME280
        """
        # BME280 address, 0x76(118)
        # Read data back from 0x88(136), 24 bytes
        b1 = self.I2C.readBytes(0x76, 0x88, 24)

        # Convert the data
        # Temp coefficents
        self.dig_T1 = b1[1] * 256 + b1[0]
        self.dig_T2 = b1[3] * 256 + b1[2]
        if self.dig_T2 > 32767 :
            self.dig_T2 -= 65536
        self.dig_T3 = b1[5] * 256 + b1[4]
        if self.dig_T3 > 32767 :
            self.dig_T3 -= 65536

        # Pressure coefficents
        self.dig_P1 = b1[7] * 256 + b1[6]
        self.dig_P2 = b1[9] * 256 + b1[8]
        if self.dig_P2 > 32767 :
            self.dig_P2 -= 65536
        self.dig_P3 = b1[11] * 256 + b1[10]
        if self.dig_P3 > 32767 :
            self.dig_P3 -= 65536
        self.dig_P4 = b1[13] * 256 + b1[12]
        if self.dig_P4 > 32767 :
            self.dig_P4 -= 65536
        self.dig_P5 = b1[15] * 256 + b1[14]
        if self.dig_P5 > 32767 :
            self.dig_P5 -= 65536
        self.dig_P6 = b1[17] * 256 + b1[16]
        if self.dig_P6 > 32767 :
            self.dig_P6 -= 65536
        self.dig_P7 = b1[19] * 256 + b1[18]
        if self.dig_P7 > 32767 :
            self.dig_P7 -= 65536
        self.dig_P8 = b1[21] * 256 + b1[20]
        if self.dig_P8 > 32767 :
            self.dig_P8 -= 65536
        self.dig_P9 = b1[23] * 256 + b1[22]
        if self.dig_P9 > 32767 :
            self.dig_P9 -= 65536

        # BME280 address, 0x76(118)
        # Read data back from 0xA1(161), 1 byte
        b1 = self.I2C.readBytes(0x76, 0xA1, 1)
        self.dig_H1 = b1[0]

        # BME280 address, 0x76(118)
        # Read data back from 0xE1(225), 7 bytes
        b1 = self.I2C.readBytes(0x76, 0xE1, 7)

        # Convert the data
        # Humidity coefficents
        self.dig_H2 = b1[1] * 256 + b1[0]
        if self.dig_H2 > 32767 :
            self.dig_H2 -= 65536
        self.dig_H3 = (b1[2] &  0xFF)
        self.dig_H4 = (b1[3] * 16) + (b1[4] & 0xF)
        if self.dig_H4 > 32767 :
            self.dig_H4 -= 65536
        self.dig_H5 = (b1[4] / 16) + (b1[5] * 16)
        if self.dig_H5 > 32767 :
            self.dig_H5 -= 65536
        self.dig_H6 = b1[6]
        if self.dig_H6 > 127 :
            self.dig_H6 -= 256

        # BME280 address, 0x76(118)
        # Select control humidity register, 0xF2(242)
        #		0x01(01)	Humidity Oversampling = 1
        self.I2C.writeByte(0x76, 0xF2, 0x01)
        # BME280 address, 0x76(118)
        # Select Control measurement register, 0xF4(244)
        #		0x27(39)	Pressure and Temperature Oversampling rate = 1
        #					Normal mode
        self.I2C.writeByte(0x76, 0xF4, 0x27)
        # BME280 address, 0x76(118)
        # Select Configuration register, 0xF5(245)
        #		0xA0(00)	Stand_by time = 1000 ms
        self.I2C.writeByte(0x76, 0xF5, 0xA0)

    def get_measurements(self):
        """
        Get Pressure from BME280
        """
        # BME280 address, 0x76(118)
        # Read data back from 0xF7(247), 8 bytes
        # Pressure MSB, Pressure LSB, Pressure xLSB, Temperature MSB, Temperature LSB
        # Temperature xLSB, Humidity MSB, Humidity LSB
        data = self.I2C.readBytes(0x76, 0xF7, 8)

        # Convert pressure and temperature data to 19-bits
        adc_p = ((data[0] * 65536) + (data[1] * 256) + (data[2] & 0xF0)) / 16
        adc_t = ((data[3] * 65536) + (data[4] * 256) + (data[5] & 0xF0)) / 16

        # Convert the humidity data
        adc_h = data[6] * 256 + data[7]

        # Temperature offset calculations
        var1 = ((adc_t) / 16384.0 - (self.dig_T1) / 1024.0) * (self.dig_T2)
        var2 = (((adc_t) / 131072.0 - (self.dig_T1) / 8192.0) * ((adc_t)/131072.0 - (self.dig_T1)/8192.0)) * (self.dig_T3)
        t_fine = (var1 + var2)
        cTemp = (var1 + var2) / 5120.0
        fTemp = cTemp * 1.8 + 32

        # Pressure offset calculations
        var1 = (t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * (self.dig_P6) / 32768.0
        var2 = var2 + var1 * (self.dig_P5) * 2.0
        var2 = (var2 / 4.0) + ((self.dig_P4) * 65536.0)
        var1 = ((self.dig_P3) * var1 * var1 / 524288.0 + ( self.dig_P2) * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * (self.dig_P1)
        p = 1048576.0 - adc_p
        p = (p - (var2 / 4096.0)) * 6250.0 / var1
        var1 = (self.dig_P9) * p * p / 2147483648.0
        var2 = p * (self.dig_P8) / 32768.0
        pressure = (p + (var1 + var2 + (self.dig_P7)) / 16.0) / 100

        # Humidity offset calculations
        var_H = ((t_fine) - 76800.0)
        var_H = (adc_h - (self.dig_H4 * 64.0 + self.dig_H5 / 16384.0 * var_H)) * (self.dig_H2 / 65536.0 * (1.0 + self.dig_H6 / 67108864.0 * var_H * (1.0 + self.dig_H3 / 67108864.0 * var_H)))
        humidity = var_H * (1.0 -  self.dig_H1 * var_H / 524288.0)
        if humidity > 100.0 :
            humidity = 100.0
        elif humidity < 0.0 :
            humidity = 0.0

        return pressure, humidity, cTemp

class BMP180(object):
    """docstring for BMP180."""

    def __init__(self, i2c):
        super(BMP180, self).__init__()
        self.I2C = i2c
        self.AC1 = 0
        self.AC2 = 0
        self.AC3 = 0
        self.AC4 = 0
        self.AC5 = 0
        self.AC6 = 0
        self.B1 = 0
        self.B2 = 0
        self.MB = 0
        self.MC = 0
        self.MD = 0

    def initialize(self):
        # BMP180 address, 0x77(119)
        # Read data back from 0xAA(170), 22 bytes
        data = self.I2C.readBytes(0x77, 0xAA, 22)

        # Convert the data
        self.AC1 = data[0] * 256 + data[1]
        if self.AC1 > 32767 :
        	self.AC1 -= 65535
        self.AC2 = data[2] * 256 + data[3]
        if self.AC2 > 32767 :
        	self.AC2 -= 65535
        self.AC3 = data[4] * 256 + data[5]
        if self.AC3 > 32767 :
        	self.AC3 -= 65535
        self.AC4 = data[6] * 256 + data[7]
        self.AC5 = data[8] * 256 + data[9]
        self.AC6 = data[10] * 256 + data[11]
        self.B1 = data[12] * 256 + data[13]
        if self.B1 > 32767 :
        	self.B1 -= 65535
        self.B2 = data[14] * 256 + data[15]
        if self.B2 > 32767 :
        	self.B2 -= 65535
        self.MB = data[16] * 256 + data[17]
        if self.MB > 32767 :
        	self.MB -= 65535
        self.MC = data[18] * 256 + data[19]
        if self.MC > 32767 :
        	self.MC -= 65535
        self.MD = data[20] * 256 + data[21]
        if self.MD > 32767 :
        	self.MD -= 65535
        pass

    def get_measurements(self):
        # BMP180 address, 0x77(119)
        # Select measurement control register, 0xF4(244)
        #		0x2E(46)	Enable temperature measurement
        self.I2C.writeByte(0x77, 0xF4, 0x2E)

        time.sleep(0.5)
        # BMP180 address, 0x77(119)
        # Read data back from 0xF6(246), 2 bytes
        # temp MSB, temp LSB
        data = self.I2C.readBytes(0x77, 0xF6, 2)

        # Convert the data
        temp = data[0] * 256 + data[1]

        # Callibration for Temperature
        X1 = (temp - self.AC6) * self.AC5 / 32768.0
        X2 = (self.MC * 2048.0) / (X1 + self.MD)
        B5 = X1 + X2
        cTemp = ((B5 + 8.0) / 16.0) / 10.0

        time.sleep(0.5)
        # BMP180 address, 0x77(119)
        # Select measurement control register, 0xF4(244)
        #		0x74(116)	Enable pressure measurement, OSS = 1
        self.I2C.writeByte(0x77, 0xF4, 0x74)

        time.sleep(0.5)
        # BMP180 address, 0x77(119)
        # Read data back from 0xF6(246), 3 bytes
        # pres MSB1, pres MSB, pres LSB
        data = self.I2C.readBytes(0x77, 0xF6, 3)

        # Convert the data
        pres = ((data[0] * 65536) + (data[1] * 256) + data[2]) / 128

        # Calibration for Pressure
        B6 = B5 - 4000
        X1 = (self.B2 * (B6 * B6 / 4096.0)) / 2048.0
        X2 = self.AC2 * B6 / 2048.0
        X3 = X1 + X2
        B3 = (((self.AC1 * 4 + X3) * 2) + 2) / 4.0
        X1 = self.AC3 * B6 / 8192.0
        X2 = (self.B1 * (B6 * B6 / 2048.0)) / 65536.0
        X3 = ((X1 + X2) + 2) / 4.0
        B4 = self.AC4 * (X3 + 32768) / 32768.0
        B7 = ((pres - B3) * (25000.0))
        pressure = 0.0
        if B7 < 2147483648L :
        	pressure = (B7 * 2) / B4
        else :
        	pressure = (B7 / B4) * 2
        X1 = (pressure / 256.0) * (pressure / 256.0)
        X1 = (X1 * 3038.0) / 65536.0
        X2 = ((-7357) * pressure) / 65536.0
        pressure = (pressure + (X1 + X2 + 3791) / 16.0) / 100

        # Calculate Altitude
        altitude = 44330 * (1 - ((pressure / 1013.25) ** 0.1903))

        return pressure, altitude, cTemp


if __name__ == '__main__':
    i2c = onionI2C.OnionI2C()
    dev1 = BME280(i2c)
    dev2 = BMP180(i2c)

    dev1.initialize()
    dev2.initialize()

    time.sleep(1)

    for x in range(0,10):
        print "//////////////////////////////////////////"
        pressure, humidity, cTemp = dev1.get_measurements()
        print "BME280"
        print "Pressure : %.2f hPa " %pressure
        print "Relative Humidity : %.2f %%" %humidity
        print "Temperature in Celsius : %.2f C" %cTemp
        time.sleep(.25)
        pressure, altitude, cTemp = dev2.get_measurements()
        print "BMP180"
        print "Pressure : %.2f hPa " %pressure
        print "Altitude : %.2f m" %altitude
        print "Temperature in Celsius : %.2f C" %cTemp

        time.sleep(1)
        pass
