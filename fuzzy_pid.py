
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

if __name__ == "__main__":


    for i in range(80):
        PID = FUZZY_PID()
        pwm_out = PID.PID_realize(80,i)
        print(pwm_out)









