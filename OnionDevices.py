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
if __name__ == '__main__':
        i2c = onionI2C.OnionI2C()
        dev1 = BME280(i2c)

        dev1.initialize()

        for x in range(0,10):
            pressure, humidity, cTemp = dev1.get_measurements()
            print "Pressure : %.2f hPa " %pressure
            print "Relative Humidity : %.2f %%" %humidity
            print "Temperature in Celsius : %.2f C" %cTemp
            time.sleep(1)
            pass
