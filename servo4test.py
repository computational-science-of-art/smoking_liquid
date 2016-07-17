# -*- coding: utf-8 -*-

import serial
import time

def convVal(m):
    return int(255.0 * m)

def makeSignal(m1, m2, m3, m4):
    return 'H' + chr(convVal(m1)) + chr(convVal(m2)) + chr(convVal(m3)) + chr(convVal(m4))

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
    print("move")
    time.sleep(1)
    for i in range(5):
        sig = makeSignal(0.0, 0.0, 0.0, 0.0)
        ser.write(sig)
        time.sleep(0.4)
        res = ser.read(5)
        print(res)
        sig = makeSignal(1.0, 1.0, 1.0, 1.0)
        ser.write(sig)
        time.sleep(0.4)
        res = ser.read(5)
        print(res)
        sig = makeSignal(0.0, 1.0, 0.0, 1.0)
        ser.write(sig)
        time.sleep(0.4)
        res = ser.read(5)
        print(res)
        sig = makeSignal(1.0, 0.0, 1.0, 0.0)
        ser.write(sig)
        time.sleep(0.4)
        res = ser.read(5)
        print(res)

    ser.close()

