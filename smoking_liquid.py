# -*- coding: utf-8 -*-

import serial
import time

class Timer():
    def __init__(self, endtarget):
        self.endtarget = endtarget

    def start(self):
        self._starttime = time.time()

    def endJudge(self):
        self._nowtime = time.time()
        self._interval = self._nowtime - self._starttime
        return self._interval >= self.endtarget

    def interval(self):
        return time.time() - self._starttime



if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
    while True:
        value = raw_input("motor value 0-255 >>> ")
        if (value.isdigit()):
            ser.write(chr(int(value)))
        elif (value == "q"):
            print("終了します")
            break
        else:
            print("error: 入力が数字ではありません")
    ser.close()

