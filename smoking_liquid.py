# -*- coding: utf-8 -*-

import serial
import time

class Timer():
    u"""動作時間カウントのためのタイマ"""
    def __init__(self, endtarget):
        self.endtarget = endtarget

    def start(self):
        u"""タイマをスタートさせる"""
        self._starttime = time.time()

    def endJudge(self):
        u"""規定の時間が経過したかどうかを判定する"""
        self._nowtime = time.time()
        self._interval = self._nowtime - self._starttime
        return self._interval >= self.endtarget

    def interval(self):
        u"""現在何秒経過したかを返す"""
        return time.time() - self._starttime


class SmokingRobot():
    u"""喫煙ロボット"""
    def __init__(self, port, startAngle=0.0, endAngle=1.0, t_breathe=2.0, t_wait=3.0, t_emit=2.0):
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.nowAngle = startAngle
        self.startAngle = startAngle
        self.endAngle = endAngle
        self.setSmokeTime(t_breathe, t_wait, t_emit)
        print("Hello! I'm a Smoking Robot.")

    def setSmokeTime(self, t_breathe, t_wait, t_emit):
        u"""喫煙（吸う，ためる，吐く）の時間を設定する"""
        self.t_breathe = t_breathe
        self.t_wait = t_wait
        self.t_emit = t_emit

    def smoke(self):
        u"""喫煙動作を実行する"""
        print("smoke start")
        self.breathe(self.t_breathe)
        time.sleep(self.t_wait)
        self.emit(self.t_emit)
        print("smoke end")

    def breathe(self, t):
        u"""吸う動作"""
        print("breathe start")
        self._moveLinear(self.endAngle, t)
        print("breathe end")

    def emit(self, t):
        u"""吐く動作"""
        print("emit start")
        self._moveLinear(self.startAngle, t)
        print("emit end")

    def _move(self, targetAngle):
        self.ser.write(chr(int(targetAngle * 255)))
        self.nowAngle = targetAngle
        print(int(targetAngle * 255))
        time.sleep(0.05)

    def _moveLinear(self, targetAngle, targetTime):
        startAngle = self.nowAngle
        timer = Timer(targetTime)
        func = lambda t: (startAngle + t / targetTime * (targetAngle - startAngle))
        print("linear motor control start")
        timer.start()
        while (not timer.endJudge()):
            self._move(func(timer.interval()))
        print("linear motor control end")

if __name__ == '__main__':
    r = SmokingRobot('/dev/ttyACM0')
    while True:
        value = raw_input("input s to smoke, input q to quit >>> ")
        if (value == "s"):
            r.smoke()
        elif (value == "q"):
            print("終了します")
            break
        else:
            print("error: 入力が適切ではありません")

