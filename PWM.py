from machine import Timer,PWM
import time

tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
ch = PWM(tim, freq=36700, duty=0, pin=26)
duty=0
dir = True
while True:

    ch.duty(80)
