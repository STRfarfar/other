from machine import I2C
import time
from ustruct import unpack, unpack_from
from array import array

############ QMCX983 config ############
QMC6983_A1_D1       = 0
QMC6983_E1          = 1
QMC7983             = 2
QMC7983_LOW_SETRESET= 3
QMC6983_E1_Metal    = 4
QMC7983_Vertical    = 5
QMC7983_Slope       = 6
# QMCX983 default qmc_address.
QMCX983_I2CADDR = 0x2C

############ BME280 config ############
# BME280 default bme_address.
BME280_I2CADDR = 0x76
# Operating Modes
BME280_OSAMPLE_1 = 1
BME280_OSAMPLE_2 = 2
BME280_OSAMPLE_4 = 3
BME280_OSAMPLE_8 = 4
BME280_OSAMPLE_16 = 5
BME280_REGISTER_CONTROL_HUM = 0xF2
BME280_REGISTER_CONTROL = 0xF4

class SPWEATHER:
    def __init__(self, i2c=None):
        if i2c is None:
            raise ValueError('An I2C object is required.')
        self.i2c = i2c
        self.bme280_init() # Temperature, humidity and pressure sensors BME280 init


    ################## BME280 ##################
    def bme280_init(self, mode=BME280_OSAMPLE_1,
                 bme_address=BME280_I2CADDR):
        # Check that mode is valid.
        if mode not in [BME280_OSAMPLE_1, BME280_OSAMPLE_2, BME280_OSAMPLE_4,
                        BME280_OSAMPLE_8, BME280_OSAMPLE_16]:
            raise ValueError(
                'Unexpected mode value {0}. Set mode to one of '
                'BME280_ULTRALOWPOWER, BME280_STANDARD, BME280_HIGHRES, or '
                'BME280_ULTRAHIGHRES'.format(mode))
        self._mode = mode
        self.bme_address = bme_address

        # load calibration data
        dig_88_a1 = self.i2c.readfrom_mem(self.bme_address, 0x88, 26)
        dig_e1_e7 = self.i2c.readfrom_mem(self.bme_address, 0xE1, 7)
        self.dig_T1, self.dig_T2, self.dig_T3, self.dig_P1, \
            self.dig_P2, self.dig_P3, self.dig_P4, self.dig_P5, \
            self.dig_P6, self.dig_P7, self.dig_P8, self.dig_P9, \
            _, self.dig_H1 = unpack("<HhhHhhhhhhhhBB", dig_88_a1)

        self.dig_H2, self.dig_H3 = unpack("<hB", dig_e1_e7)
        e4_sign = unpack_from("<b", dig_e1_e7, 3)[0]
        self.dig_H4 = (e4_sign << 4) | (dig_e1_e7[4] & 0xF)

        e6_sign = unpack_from("<b", dig_e1_e7, 5)[0]
        self.dig_H5 = (e6_sign << 4) | (dig_e1_e7[4] >> 4)

        self.dig_H6 = unpack_from("<b", dig_e1_e7, 6)[0]

        self.i2c.writeto_mem(self.bme_address, BME280_REGISTER_CONTROL,
                             bytearray([0x3F]))
        self.t_fine = 0

        # temporary data holders which stay allocated
        self._l1_barray = bytearray(1)
        self._l8_barray = bytearray(8)
        self._l3_resultarray = array("i", [0, 0, 0])

    def read_raw_data(self, result):
        """ Reads the raw (uncompensated) data from the sensor.
            Args:
                result: array of length 3 or alike where the result will be
                stored, in temperature, pressure, humidity order
            Returns:
                None
        """

        self._l1_barray[0] = self._mode
        self.i2c.writeto_mem(self.bme_address, BME280_REGISTER_CONTROL_HUM,
                             self._l1_barray)
        self._l1_barray[0] = self._mode << 5 | self._mode << 2 | 1
        self.i2c.writeto_mem(self.bme_address, BME280_REGISTER_CONTROL,
                             self._l1_barray)

        sleep_time = 1250 + 2300 * (1 << self._mode)
        sleep_time = sleep_time + 2300 * (1 << self._mode) + 575
        sleep_time = sleep_time + 2300 * (1 << self._mode) + 575
        time.sleep_us(sleep_time)  # Wait the required time

        # burst readout from 0xF7 to 0xFE, recommended by datasheet
        self.i2c.readfrom_mem_into(self.bme_address, 0xF7, self._l8_barray)
        readout = self._l8_barray
        # pressure(0xF7): ((msb << 16) | (lsb << 8) | xlsb) >> 4
        raw_press = ((readout[0] << 16) | (readout[1] << 8) | readout[2]) >> 4
        # temperature(0xFA): ((msb << 16) | (lsb << 8) | xlsb) >> 4
        raw_temp = ((readout[3] << 16) | (readout[4] << 8) | readout[5]) >> 4
        # humidity(0xFD): (msb << 8) | lsb
        raw_hum = (readout[6] << 8) | readout[7]

        result[0] = raw_temp
        result[1] = raw_press
        result[2] = raw_hum

    def read_compensated_data(self, result=None):
        """ Reads the data from the sensor and returns the compensated data.
            Args:
                result: array of length 3 or alike where the result will be
                stored, in temperature, pressure, humidity order. You may use
                this to read out the sensor without allocating heap memory
            Returns:
                array with temperature, pressure, humidity. Will be the one from
                the result parameter if not None
        """
        self.read_raw_data(self._l3_resultarray)
        raw_temp, raw_press, raw_hum = self._l3_resultarray
        # temperature
        var1 = ((raw_temp >> 3) - (self.dig_T1 << 1)) * (self.dig_T2 >> 11)
        var2 = (((((raw_temp >> 4) - self.dig_T1) *
                  ((raw_temp >> 4) - self.dig_T1)) >> 12) * self.dig_T3) >> 14
        self.t_fine = var1 + var2
        temp = (self.t_fine * 5 + 128) >> 8

        # pressure
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * self.dig_P6
        var2 = var2 + ((var1 * self.dig_P5) << 17)
        var2 = var2 + (self.dig_P4 << 35)
        var1 = (((var1 * var1 * self.dig_P3) >> 8) +
                ((var1 * self.dig_P2) << 12))
        var1 = (((1 << 47) + var1) * self.dig_P1) >> 33
        if var1 == 0:
            pressure = 0
        else:
            p = 1048576 - raw_press
            p = (((p << 31) - var2) * 3125) // var1
            var1 = (self.dig_P9 * (p >> 13) * (p >> 13)) >> 25
            var2 = (self.dig_P8 * p) >> 19
            pressure = ((p + var1 + var2) >> 8) + (self.dig_P7 << 4)

        # humidity
        h = self.t_fine - 76800
        h = (((((raw_hum << 14) - (self.dig_H4 << 20) -
                (self.dig_H5 * h)) + 16384)
              >> 15) * (((((((h * self.dig_H6) >> 10) *
                            (((h * self.dig_H3) >> 11) + 32768)) >> 10) +
                          2097152) * self.dig_H2 + 8192) >> 14))
        h = h - (((((h >> 15) * (h >> 15)) >> 7) * self.dig_H1) >> 4)
        h = 0 if h < 0 else h
        h = 419430400 if h > 419430400 else h
        humidity = h >> 12

        if result:
            result[0] = temp
            result[1] = pressure
            result[2] = humidity
            return result

        return array("i", (temp, pressure, humidity))

    @property
    def bme_values(self):
        """ human readable values """

        t, p, h = self.read_compensated_data()

        p = p // 256
        pi = p // 100
        pd = p - pi * 100

        hi = h // 1024
        hd = h * 100 // 1024 - hi * 100
        shidu = hi + hd
        return (t / 100, pi + pd, hi + hd)
        #return ("{}C".format(t / 100), "{}.{:02d}hPa".format(pi, pd),"{}.{:02d}%".format(hi, hd))

    ################## BME280 End ##################
import math


#####################Fuzzy-PID########################

#误差的阀值，小于这个数值的时候，不做PID调整，避免误差较小时频繁调节引起震荡
Emin = 0
Emid = 0.08
Emax = 0.6
#调整值限幅，防止积分饱和
Umax = 6
Umin = -6
#输出值限幅
Pmax = 1000
Pmin = 0


NB = 0
NM = 1
NS = 2
ZO = 3
PS = 4
PM = 5
PB = 6
KP = [
    [PB, PB, PM, PM, PS, ZO, ZO],
    [PB, PB, PM, PS, PS, ZO, NS],
    [PM, PM, PM, PS, ZO, NS, NS],
    [PM, PM, PS, ZO, NS, NM, NM],
    [PS, PS, ZO, NS, NS, NM, NM],
    [PS, ZO, PS, NM, NM, NM, NB],
    [ZO, ZO, NM, NM, NM, NB, NB]
]
KD = [
    [PS, NS, NB, NB, NB, NM, PS],
    [PS, NS, NB, NM, NM, NS, ZO],
    [ZO, NS, NM, NM, NS, NS, ZO],
    [ZO, NS, NS, NS, NS, NS, ZO],
    [ZO, ZO, ZO, ZO, ZO, ZO, ZO],
    [PB, ZO, PS, PS, PS, PS, PB],
    [PB, PM, PM, PM, PS, PS, PB],
]
KI = [
    [NB, NB, NM, NM, NS, ZO, ZO],
    [NB, NB, NM, NS, NS, ZO, ZO],
    [NB, NM, NS, NS, ZO, PS, PS],
    [NM, NM, NS, ZO, PS, PM, PM],
    [NM, NS, ZO, PS, PS, PM, PB],
    [ZO, ZO, PS, PS, PM, PB, PB],
    [ZO, ZO, PS, PM, PM, PB, PB]
]
class FUZZY_PID:
    Kp = Ki = Kd = T= K1 = K2 = K3 = LastError = PrevError = pwn_out = 0.0
    flag = 1
    #**************求隶属度（三角形）***************
    def FTri(self, x, a, b, c): #FuzzyTriangle
        if x <= a:
            return 0
        elif (a < x) and (x <= b):
            return (x-a)/(b-a)
        elif (b < x) and (x <= c):
            return (c - x) / (c - b)
        elif x>c:
            return 0
        else:
            return 0


    #*****************求隶属度（梯形左）*******************
    def FTraL(self, x, a, b): #FuzzyTrapezoidRight
        if x <= a:
            return 1
        elif (a < x) and (x <= b):
            return (b - x) / (b - a)
        elif x > b:
            return 0
        else:
            return 0

    #*****************求隶属度（梯形右）*******************
    def FTraR(self, x, a, b):
        if x <= a:
            return 0
        if (a < x) and (x <= b):
            return 1
        if (x >= b):
            return 1
        else:
            return 1



    #****************三角形反模糊化处理**********************
    def uFTri(self, x, a, b, c):
        z = (b - a) * x + a
        y = c - (c - b) * x
        return (y + z) / 2

    #*******************梯形（左）反模糊化***********************
    def uFTraL(self, x, a, b):
        return b - (b - a) * x


    #*******************梯形（右）反模糊化***********************
    def uFTraR(self, x, a, b):
        return (b - a) * x + a

    #**************************求交集***************************
    def fand(self, a, b):
        return a if (a < b) else b

    #**************************求并集****************************
    def forr(self, a, b):
        return b if (a < b) else a

    #==========   PID计算部分   ======================
    def PID_realize(self, s, i):
        pwm_var = 0.0
        #输入格式的转化及偏差计算
        set = float(s / 100)
        input = float(i / 100)
        iError = set - input #偏差
        es = [0,0,0,0,0,0,0]
        ecs = [0,0,0,0,0,0,0]
        form = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        e = iError
        ec = iError - self.LastError
        MaxX = 0
        MaxY = 0
        #当温度差的绝对值小于Emax时，对pid的参数进行调整
        if abs(iError) <= Emax:
            es[NB]=self.FTraL(e*5,-3,-1)
            es[NM]=self.FTri(e*5,-3,-2,0)
            es[NS]=self.FTri(e*5,-3,-1,1)
########################Fuzzy-PID END#########################
#误差的阀值，小于这个数值的时候，不做PID调整，避免误差较小时频繁调节引起震荡
Emin = 0
Emid = 0.08
Emax = 0.6
#调整值限幅，防止积分饱和
Umax = 6
Umin = -6
#输出值限幅
Pmax = 7200
Pmin = 0


NB = 0
NM = 1
NS = 2
ZO = 3
PS = 4
PM = 5
PB = 6
KP = [
    [PB, PB, PM, PM, PS, ZO, ZO],
    [PB, PB, PM, PS, PS, ZO, NS],
    [PM, PM, PM, PS, ZO, NS, NS],
    [PM, PM, PS, ZO, NS, NM, NM],
    [PS, PS, ZO, NS, NS, NM, NM],
    [PS, ZO, PS, NM, NM, NM, NB],
    [ZO, ZO, NM, NM, NM, NB, NB]
]
KD = [
    [PS, NS, NB, NB, NB, NM, PS],
    [PS, NS, NB, NM, NM, NS, ZO],
    [ZO, NS, NM, NM, NS, NS, ZO],
    [ZO, NS, NS, NS, NS, NS, ZO],
    [ZO, ZO, ZO, ZO, ZO, ZO, ZO],
    [PB, ZO, PS, PS, PS, PS, PB],
    [PB, PM, PM, PM, PS, PS, PB],
]
KI = [
    [NB, NB, NM, NM, NS, ZO, ZO],
    [NB, NB, NM, NS, NS, ZO, ZO],
    [NB, NM, NS, NS, ZO, PS, PS],
    [NM, NM, NS, ZO, PS, PM, PM],
    [NM, NS, ZO, PS, PS, PM, PB],
    [ZO, ZO, PS, PS, PM, PB, PB],
    [ZO, ZO, PS, PM, PM, PB, PB]
]
class FUZZY_PID:
    Kp = Ki = Kd = T= K1 = K2 = K3 = LastError = PrevError = pwn_out = 0.0
    flag = 1
    #**************求隶属度（三角形）***************
    def FTri(self, x, a, b, c): #FuzzyTriangle
        if x <= a:
            return 0
        elif (a < x) and (x <= b):
            return (x-a)/(b-a)
        elif (b < x) and (x <= c):
            return (c - x) / (c - b)
        elif x>c:
            return 0
        else:
            return 0


            #*****************求隶属度（梯形左）*******************
    def FTraL(self, x, a, b): #FuzzyTrapezoidRight
        if x <= a:
            return 1
        elif (a < x) and (x <= b):
            return (b - x) / (b - a)
        elif x > b:
            return 0
        else:
            return 0

            #*****************求隶属度（梯形右）*******************
    def FTraR(self, x, a, b):
        if x <= a:
            return 0
        if (a < x) and (x <= b):
            return 1
        if (x >= b):
            return 1
        else:
            return 1



            #****************三角形反模糊化处理**********************
    def uFTri(self, x, a, b, c):
        z = (b - a) * x + a
        y = c - (c - b) * x
        return (y + z) / 2

        #*******************梯形（左）反模糊化***********************
    def uFTraL(self, x, a, b):
        return b - (b - a) * x


        #*******************梯形（右）反模糊化***********************
    def uFTraR(self, x, a, b):
        return (b - a) * x + a

        #**************************求交集***************************
    def fand(self, a, b):
        return a if (a < b) else b

        #**************************求并集****************************
    def forr(self, a, b):
        return b if (a < b) else a

        #==========   PID计算部分   ======================
    def PID_realize(self, s, i):
        pwm_var = 0.0
        #输入格式的转化及偏差计算
        set = float(s / 100)
        input = float(i / 100)
        iError = set - input #偏差
        es = [0,0,0,0,0,0,0]
        ecs = [0,0,0,0,0,0,0]
        form = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        e = iError
        ec = iError - self.LastError
        MaxX = 0
        MaxY = 0
        #当温度差的绝对值小于Emax时，对pid的参数进行调整
        if abs(iError) <= Emax:
            es[NB]=self.FTraL(e*5,-3,-1)
            es[NM]=self.FTri(e*5,-3,-2,0)
            es[NS]=self.FTri(e*5,-3,-1,1)
            es[ZO]=self.FTri(e*5,-2,0,2)
            es[PS]=self.FTri(e*5,-1,1,3)
            es[PM]=self.FTri(e*5,0,2,3)
            es[PB]=self.FTraR(e*5,1,3)

            ecs[NB]=self.FTraL(ec*30,-3,-1)
            ecs[NM]=self.FTri(ec*30,-3,-2,0)
            ecs[NS]=self.FTri(ec*30,-3,-1,1)
            ecs[ZO]=self.FTri(ec*30,-2,0,2)
            ecs[PS]=self.FTri(ec*30,-1,1,3)
            ecs[PM]=self.FTri(ec*30,0,2,3)
            ecs[PB]=self.FTraR(ec*30,1,3)

            for i in range(7):
                for j in range(7):
                    form[i][j] = (self.fand(es[i], ecs[j]))

            for i in range(7):
                for j in range(7):
                    if form[MaxX][MaxY] < form[i][j]:
                        MaxX = i
                        MaxY = j
            lsd = form[MaxX][MaxY]
            temp_p = KP[MaxX][MaxY]
            temp_d = KD[MaxX][MaxY]
            temp_i = KI[MaxX][MaxY]

            if temp_p == NB:
                detkp = self.uFTraL(lsd, -0.3, -0.1)

            elif temp_p == NM:
                detkp = self.uFTri(lsd, -0.3, -0.2, 0)
            elif temp_p == NS:
                detkp = self.uFTri(lsd,-0.3,-0.1,0.1)
            elif temp_p == ZO:
                detkp = self.uFTri(lsd,-0.2,0,0.2)
            elif temp_p == PS:
                detkp = self.uFTri(lsd,-0.1,0.1,0.3)
            elif temp_p == PM:
                detkp = self.uFTri(lsd,0,0.2,0.3)
            elif temp_p == PB:
                detkp = self.uFTraR(lsd,0.1,0.3)

            if temp_d == NB:
                detkd = self.uFTraL(lsd,-3,-1)
            elif temp_d == NM:
                detkd = self.uFTri(lsd,-3,-2,0)
            elif temp_d == NS:
                detkd = self.uFTri(lsd,-3,1,1)
            elif temp_d == ZO:
                detkd = self.uFTri(lsd,-2,0,2)
            elif temp_d==PS:
                detkd = self.uFTri(lsd,-1,1,3)
            elif temp_d==PM:
                detkd = self.uFTri(lsd,0,2,3)
            elif temp_d==PB:
                detkd = self.uFTraR(lsd,1,3)

            if temp_i == NB:
                detki = self.uFTraL(lsd,-0.06,-0.02)
            elif temp_i==NM:
                detki = self.uFTri(lsd,-0.06,-0.04,0)
            elif temp_i == NS:
                detki = self.uFTri(lsd,-0.06,-0.02,0.02)
            elif temp_i == ZO:
                detki = self.uFTri(lsd,-0.04,0,0.04)
            elif temp_i==PS:
                detki = self.uFTri(lsd,-0.02,0.02,0.06)
            elif temp_i==PM:
                detki = self.uFTri(lsd,0,0.04,0.06)
            elif temp_i==PB:
                detki = self.uFTraR(lsd,0.02,0.06)


                #pid三项系数的修改
            self.Kp += detkp
            self.Ki += detki
            self.Kd += detkd

            #对Kp,Ki,Kd进行限幅
            if self.Kp < 0:
                self.Kp = 0
            if self.Kd < 0:
                self.Kd = 0
            if self.Ki < 0:
                self.Ki = 0
                #计算新的K1,K2,K3v
            self.K1 = self.Kp + self.Ki + self.Kd
            self.Ke = self.Kp + self.Kd * 2
            self.K3 = self.Kd

        if iError > Emax:
            self.pwm_out = 7200
            self.pwm_var = 0
            self.flag = 1 #设定标志位，如果误差超过了门限值，则认为当控制量第一次到达给定值时，应该采取下面的 抑制超调 的措施
        elif iError < -Emax:
            self.pwm_out = 0
            pwm_var = 0
        elif abs(iError) < Emin:
            self.pwm_var = -1
        else:
            if (iError < Emid) and (self.flag == 1):
                self.pwm_out = 0
                self.flag = 0
            elif -iError > Emid:
                self.pwm_var = -1
            else:
                self.pwm_var = self.K1 * iError + self.K2 * self.LastError + self.K3 * self.PrevError
            if self.pwm_var >= Umax : self.pwm_var = Umax

            if self.pwm_var <= Umin : self.pwm_var = Umin

        self.PrevError = self.LastError
        self.LastError = iError


        self.pwm_out += 360 * self.pwm_var

        if self.pwm_out > Pmax:
            self.pwm_out = Pmax

        if self.pwm_out < Pmin:
            self.pwm_out = Pmin


        return self.pwm_out

    def PID_set(self, Kp, Ki, Kd, T):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.T = T
        self.K1 = self.Kp * (1 + self.Ki + self.Kd)
        self.K2 = -(self.Kp + 2 * self.Kp * self.Kd)
        self.K3 = self.Kp * self.Kd

    def __init__(self):
        #self.PID_set(1, 2, 3, 1)
        self.flag = 0
        self.pwm_out = 0


    ################## SP_WEATHER Demo ##################
if __name__ == "__main__":
    from machine import I2C
    from fpioa_manager import fm
    import time

    ############# config #############
    WEATHER_I2C_NUM_1 = I2C.I2C1
    WEATHER_I2C_FREQ_KHZ_1 = 100
    WEATHER_I2C_SCL_1 = 27
    WEATHER_I2C_SDA_1 = 26
    ##################################

    WEATHER_I2C_NUM_2 = I2C.I2C0
    WEATHER_I2C_FREQ_KHZ_2 = 100
    WEATHER_I2C_SCL_2 = 25
    WEATHER_I2C_SDA_2 = 24

    i2c_bus_1 = I2C(WEATHER_I2C_NUM_1, freq=WEATHER_I2C_FREQ_KHZ_1*1000,
    scl=WEATHER_I2C_SCL_1, sda=WEATHER_I2C_SDA_1, gscl = fm.fpioa.GPIOHS26,
    gsda = fm.fpioa.GPIOHS27)
    i2c_devs_list_1 = i2c_bus_1.scan()
    '''
    i2c_bus_2 = I2C(WEATHER_I2C_NUM_1, freq=WEATHER_I2C_FREQ_KHZ_1*1000,
    scl=WEATHER_I2C_SCL_1, sda=WEATHER_I2C_SDA_1, gscl = fm.fpioa.GPIOHS24,
    gsda = fm.fpioa.GPIOHS25)
    i2c_devs_list_2 = i2c_bus_2.scan()

    print("I2C devices 1:" + str(i2c_devs_list_1))
    print("I2C devices 2:" + str(i2c_devs_list_2))
    '''
    weather1=SPWEATHER(i2c=i2c_bus_1) # create sp_weather
    #weather2=SPWEATHER(i2c=i2c_bus_2)

    while 1:
        time.sleep_ms(500)
        PID = FUZZY_PID()
        PWM = PID.PID_realize(80,weather1.bme_values[0])
        print(PWM)
        print((1000-PWM)/10)
        print(weather1.bme_values[0])
        #print(weather2.bme_values[0])
        #print(weather.bme_values[2])
