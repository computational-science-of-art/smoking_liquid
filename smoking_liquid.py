# -*- coding: utf-8 -*-

import serial
import time

def makeSignal(header, data):
    u"""headerはchar1文字,dataは0.0-1.0のfloat"""
    buf = header
    for val in data:
        buf += chr(int(val * 255))
    return buf

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
    def __init__(self, port,
            startAngle=[0.0, 0.0, 0.0],
            endAngle=[1.0, 1.0, 1.0],
            cigarSetAngle=[0.0, 1.0, 1.0],
            cigarUnsetAngle=[1.0, 0.0, 0.0],
            t_breathe=2.0, t_wait=3.0, t_emit=2.0, t_set=1.5,
            header='H'):
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        # headerはシリアル通信の始まりを示す1文字
        self.header = header
        self.nowAngle = startAngle
        self.startAngle = startAngle
        self.endAngle = endAngle
        self.cigarSetAngle = cigarSetAngle
        self.cigarUnsetAngle = cigarUnsetAngle
        self.setSmokeTime(t_breathe, t_wait, t_emit)
        self.t_set = t_set
        self._move(self.nowAngle)
        print("Initializing ...")
        time.sleep(2)
        print("Hello! I'm a Smoking Robot.")

    def setSmokeTime(self, t_breathe, t_wait, t_emit):
        u"""喫煙（吸う，ためる，吐く）の時間を設定する"""
        self.t_breathe = t_breathe
        self.t_wait = t_wait
        self.t_emit = t_emit

    def smoke(self):
        u"""喫煙動作を実行する"""
        print("smoke start")
        self.setCigar(self.t_set)
        self.breathe(self.t_breathe)
        time.sleep(self.t_wait)
        self.unsetCigar(self.t_set)
        self.emit(self.t_emit)
        print("smoke end")

    def setCigar(self, t):
        u"""タバコを口元に持っていく動作"""
        print("set the cigar")
        self._moveLinear(self.cigarSetAngle, t)
        print("set the cigar complete")

    def unsetCigar(self, t):
        u"""タバコを口元から離す動作"""
        print("unset the cigar")
        self._moveLinear(self.cigarUnsetAngle, t)
        print("unset the cigar complete")

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
        self.ser.write(makeSignal(self.header, targetAngle))
        self.nowAngle = targetAngle
        time.sleep(0.02)
        print(self.nowAngle)

    def _moveLinear(self, targetAngle, targetTime):
        startAngle = self.nowAngle
        timer = Timer(targetTime)
        func = lambda t, sa, ta: (sa + t / targetTime * (ta - sa))
        print("linear motor control start")
        timer.start()
        while (not timer.endJudge()):
            self._move(map(func, [timer.interval()] * len(startAngle),
                startAngle, targetAngle))
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

