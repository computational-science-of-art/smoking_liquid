# -*- coding: utf-8 -*-

import serial
import time

delay = 1.0

def convVal(m):
    return int(255.0 * m)

def makeSignal(m1, m2, m3, m4):
    return 'H' + chr(convVal(m1)) + chr(convVal(m2)) + chr(convVal(m3)) + chr(convVal(m4))

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
    print("move")
    time.sleep(1)
    for i in range(2):
        sig = makeSignal(0.2, 0.2, 0.2, 0.2)
        print(sig)
        ser.write(sig)
        time.sleep(delay)
        res = ser.read(5)
        print(res)
        sig = makeSignal(0.8, 0.8, 0.6, 0.6)
        print(sig)
        ser.write(sig)
        time.sleep(delay)
        res = ser.read(5)
        print(res)

    ser.close()

