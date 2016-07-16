# -*- coding: utf-8 -*-

import serial

def convVal(m):
    return int(255.0 * m)

def makeSignal(m1, m2, m3):
    return 'H' + chr(convVal(m1)) + chr(convVal(m2)) + chr(convVal(m3))

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
    sig = makeSignal(0.0, 0.0, 0.0)
    ser.write(sig)
    res = ser.read(5)
    print(res)
    sig = makeSignal(1.0, 1.0, 1.0)
    ser.write(sig)
    res = ser.read(5)
    print(res)
    sig = makeSignal(0.0, 1.0, 0.0)
    ser.write(sig)
    res = ser.read(5)
    print(res)
    sig = makeSignal(1.0, 0.0, 1.0)
    ser.write(sig)
    res = ser.read(5)
    print(res)

    ser.close()

