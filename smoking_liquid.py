# -*- coding: utf-8 -*-

import serial
import time
import numpy as np
from math import sin, cos, pi


def makeSignal(header, data):
    u"""headerはchar1文字,dataは0.0-1.0のfloat"""
    buf = header
    for val in data:
        if (val > 1.0 or val < 0.0):
            val = max(min(val, 1.0), 0.0)
            print("invalid targetAngle")
        buf += chr(int(val * 255))
    return buf

def linearInterpolation(t, targetTime, initialValue, targetValue):
    u"""線形補間"""
    return initialValue + t / targetTime * (targetValue - initialValue)


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
            initialState=np.array([0.0, 0.0, 0.1, 0.22]),
            endState=np.array([0.0, 0.0, 0.3, 0.42]),
            t_breathe=1.0, t_wait=1.0, t_emit=1.0, t_set=1.5,
            handlink=np.array([59, 51, 59]),
            handDistance=np.array([20.0, 0.0, 0.0]),
            servoRotation=[True, True, False, True],
            header='H'):
        # [0 0 0.18 0]
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        # headerはシリアル通信の始まりを示す1文字
        self.header = header
        self.nowAngle = initialState
        self.initialState = initialState
        self.endState = endState
        self.cigarSetState = False
        self.breatheState = False
        self.setSmokeTime(t_breathe, t_wait, t_emit)
        self.t_set = t_set
        self.hand = Hand(handlink, self.nowAngle * pi)
        self.handDistance = handDistance
        self.servoRotation = servoRotation
        self._move(self.nowAngle)
        print("Initializing ...")
        time.sleep(1)
        print("Hello! I'm a Smoking Robot.")

    def setSmokeTime(self, t_breathe, t_wait, t_emit):
        u"""喫煙（吸う，ためる，吐く）の時間を設定する"""
        self.t_breathe = t_breathe
        self.t_wait = t_wait
        self.t_emit = t_emit

    def _move(self, targetAngle, t=0.02):
        u"""全関節を動かす"""
        #convertedTarget = self.convertServoRotation(targetAngle)
        convertedTarget = targetAngle
        self.ser.write(makeSignal(self.header, convertedTarget))
        self.nowAngle = targetAngle
        time.sleep(t)
        print(self.nowAngle)

    def convertServoRotation(self, targetAngle):
        u"""サーボの回転方向を設定"""
        result = targetAngle
        for (i, val) in enumerate(self.servoRotation):
            if (not val):
                result[i] = 1.0 - targetAngle[i]
        return result

    def _moveOneServo(self, i, targetAngle):
        u"""一つのサーボのみを動かす"""
        targetAngleVector = self.nowAngle
        targetAngleVector[i] = targetAngle
        self._move(targetAngleVector)

    def smoke(self):
        u"""喫煙動作を実行する"""
        self.cigarSetState = False
        self.breatheState = False
        print("smoke start")
        self.setCigar(self.t_set)
        self._moveBreathe(self.t_breathe)
        time.sleep(self.t_wait)
        self.setCigar(self.t_set)
        self._moveBreathe(self.t_emit)
        print("smoke end")

    def _moveBreathe(self, t):
        u"""状態に応じて吸ったり吐いたりする"""
        if (self.breatheState):
            # 吐く
            startAngle = 1.0
            targetAngle = 0.0
        else:
            # 吸う
            startAngle = 0.0
            targetAngle = 1.0
        timer = Timer(t)
        timer.start()
        while (not timer.endJudge()):
            self._moveOneServo(0,
                    linearInterpolation(timer.interval(), t, startAngle, targetAngle))
        self.breatheState = not self.breatheState

    def setCigar(self, t):
        u"""状態に応じてタバコを口元に持っていく/口元から離す"""
        if (self.cigarSetState):
            # 離す
            print("unset the cigar")
            dx = -self.handDistance
            targetAngle = self.initialState
        else:
            # 近づける
            print("set the cigar")
            dx = self.handDistance
            targetAngle = self.endState
        targetAngle[0] = self.nowAngle[0]
        self._moveLinear(targetAngle, self.t_set)
        #self._moveHand(dx, t)
        self.cigarSetState = not self.cigarSetState
        print("cigar positioning complete")

    def _moveHand(self, dx, t):
        u"""手を動かす"""
        step = 100
        delay = float(t) / step
        xStep = dx / step
        for i in range(step):
            # targetAngleは0.0-1.0で指定なのでpiで割る
            targetAngle = list(self.hand.ikStep(xStep) / pi)
            targetAngleVector = np.r_[np.array([self.nowAngle[0]]), targetAngle]
            self._move(targetAngleVector, delay)

    def _moveLinear(self, targetAngle, targetTime):
        u"""線形補間しながら全関節を動かす"""
        startAngle = self.nowAngle
        timer = Timer(targetTime)
        func = lambda t, sa, ta: (sa + t / targetTime * (ta - sa))
        print("linear motor control start")
        timer.start()
        while (not timer.endJudge()):
            self._move(map(func, [timer.interval()] * len(startAngle),
                startAngle, targetAngle))
        print("linear motor control end")


class Hand():
    def __init__(self, link, initialAngle):
        self.link = link
        self.joint = initialAngle[1:4]
        print("handAngle = " + str(self.joint))

    def jacobi(self, s):
        l = self.link
        jacobi = np.array([
            [- l[0] * sin(s[0] - np.deg2rad(10)) - l[1] * sin(s[0] + s[1] - np.deg2rad(10)) - l[2] * sin(s[0] + s[1] + s[2] + np.deg2rad(10)),
             - l[1] + sin(s[0] + s[1] - np.deg2rad(10)) - l[2] * sin(s[0] + s[1] + s[2] + np.deg2rad(10)),
             - l[2] * sin(s[0] + s[1] + s[2] + np.deg2rad(10))],
            [l[0] * cos(s[0] - np.deg2rad(10)) + l[1] * cos(s[0] + s[1] - np.deg2rad(10)) + l[2] * cos(s[0] + s[1] + s[2] + np.deg2rad(10)),
             l[1] + cos(s[0] + s[1] - np.deg2rad(10)) + l[2] * cos(s[0] + s[1] + s[2] + np.deg2rad(10)),
             l[2] * cos(s[0] + s[1] + s[2] + np.deg2rad(10))],
            [1, 1, 1]])
        return jacobi

    def ikStep(self, dp):
        j = self.jacobi(self.joint)
        self.joint = self.joint + np.dot(np.linalg.inv(j), dp)
        return self.joint


if __name__ == '__main__':
    r = SmokingRobot('/dev/ttyACM0')
    while True:
        value = raw_input("input s to smoke, input q to quit >>> ")
        if (value == "s"):
            r.smoke()
        elif (value == "q"):
            print("終了します")
            break
        elif (value == "b"):
            r._moveBreathe(r.t_set)
        elif (value == "h"):
            r.setCigar(1.0)
        else:
            print("error: 入力が適切ではありません")

