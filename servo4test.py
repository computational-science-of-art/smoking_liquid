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
    while True:
        value = raw_input("motor value 0-255 >>> ")
        vals = map(float, value.split(" "))
        print(vals)
        if (not len(vals) == 4):
            print("vals length error")
            continue
        sig = makeSignal(vals[0], vals[1], vals[2], vals[3])
        ser.write(sig)
    ser.close()

