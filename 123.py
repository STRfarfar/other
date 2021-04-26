from machine import Timer,PWM
import time

#PWM通过定时器配置，接到IO17引脚（Pin IO17）
tim1 = Timer(Timer.TIMER1, Timer.CHANNEL1, mode=Timer.MODE_PWM)
tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
S1 = PWM(tim1, freq=50, duty=0, pin=24)
S2 = PWM(tim, freq=50, duty=0, pin=26)

'''
说明：舵机控制函数
功能：180度舵机：angle:-90至90 表示相应的角度
     360连续旋转度舵机：angle:-90至90 旋转方向和速度值。
    【duty】占空比值：0-100
'''

def Servo(servo,angle):
    servo.duty((angle+90)/180*10+2.5)


while True:
    #-90度
    Servo(S1,-75)
    Servo(S2,-75)
    time.sleep(1)

    #-45度
    Servo(S1,-45)
    Servo(S2,-45)
    time.sleep(1)

    #0度
    Servo(S1,0)
    Servo(S2,-0)
    time.sleep(1)

    #45度
    Servo(S1,45)
    Servo(S2,45)
    time.sleep(1)

    #90度
    Servo(S1,75)
    Servo(S2,75)
    time.sleep(1)