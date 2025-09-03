#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意demo无法直接运行，需要pip安装sdk后才能运行
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    while (not piper.EnablePiper()):
        time.sleep(0.01)
    #piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
    #piper.JointCtrl(3240, 92141, -22369, 9926, -49349, -92803)
    piper.MotionCtrl_2(0x01, 0x00, 100, 0x00)
    piper.EndPoseCtrl(257384, 4793, 147021, -105689, 1512, -94993)
    X_axis: 262919
    Y_axis: 2962
    Z_axis: 161703
    RX_axis: -105689
    RY_axis: 1512
    RZ_axis: -94993
    piper.GripperCtrl(0, 1000, 0x01, 0)  # enable gripper position 0

