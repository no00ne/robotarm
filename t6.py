#!/usr/bin/env python3
"""
Piper机械臂归零及直线运动示例脚本
功能：1.txt. 使能机械臂并归零各关节
      2. 读取当前末端位姿
      3. 计算Z轴上移20cm后的目标位姿
      4. 以直线运动方式(MOVE L)移动至目标位姿
基于piper_sdk库：https://github.com/agilexrobotics/piper_sdk
参考接口文档：https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/V2/INTERFACE_V2.MD
"""

import time
import can
from piper_sdk import *

# 机械臂归零函数
def home_arm(piper):
    """
    将机械臂所有关节归零（回到零点位置）
    """
    print("开始机械臂归零...")
    # 假设归零位置是各关节为0的角度（单位：度）
    # 实际零点位置可能需要根据你的机械臂型号和校准情况调整
    zero_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 6个关节的归零目标角度

    # 使用关节空间控制执行归零运动
    # JointCtrl接口用于控制关节角度:cite[7]
    piper.JointCtrl(zero_joints, speed=0.5) # speed参数控制运动速度比例（0.0到1.0）

    # 等待运动完成（这里采用简单等待，实际生产环境应通过状态查询判断）
    time.sleep(5) # 等待时间需根据实际运动距离调整
    print("机械臂归零完成。")

# 主控制函数
def main():
    # 初始化PiperSDK
    piper = PiperSDK()

    try:
        # 1.txt. 连接CAN端口
        # 假设使用can0接口，比特率1M:cite[1.txt]:cite[2]
        print("正在连接CAN端口...")
        piper.ConnectPort('can0', 1000000)
        time.sleep(1) # 短暂等待连接稳定

        # 2. 使能机械臂:cite[1.txt]:cite[2]
        print("使能机械臂...")
        piper.EnableArm() # 使能机械臂电机
        time.sleep(1)     # 等待使能完成

        # 3. 执行归零操作
        home_arm(piper)

        # 4. 读取当前末端执行器位姿（TCP位姿）:cite[1.txt]:cite[7]
        # 位姿通常表示为 [X, Y, Z, RX, RY, RZ] (单位：毫米和度)
        print("读取当前末端位姿...")
        current_pose = piper.GetArmEndPoseMsgs()
        print(f"当前末端位姿: X={current_pose[0]:.2f}mm, Y={current_pose[1]:.2f}mm, Z={current_pose[2]:.2f}mm, "
              f"RX={current_pose[3]:.2f}°, RY={current_pose[4]:.2f}°, RZ={current_pose[5]:.2f}°")

        # 5. 计算目标位姿：Z轴上移20cm (200mm)
        target_pose = current_pose.copy() # 复制当前位姿
        target_pose[2] += 200.0 # 修改Z坐标

        # 6. 使用EndPoseCtrl接口控制末端执行器以直线运动方式移动到目标位姿:cite[7]
        # 0x02: MOVE L (Linear) 直线运动方式:cite[4]:cite[10]
        print("执行直线运动，末端上移20cm...")
        piper.EndPoseCtrl(target_pose, speed=0.3, move_type=0x02) # move_type=0x02表示直线运动

        # 等待直线运动完成
        time.sleep(5) # 等待时间需根据实际运动距离和速度调整
        print("直线运动完成。")

        # 7. （可选）再次读取并显示移动后的位姿
        new_pose = piper.GetArmEndPoseMsgs()
        print(f"移动后末端位姿: X={new_pose[0]:.2f}mm, Y={new_pose[1]:.2f}mm, Z={new_pose[2]:.2f}mm, "
              f"RX={new_pose[3]:.2f}°, RY={new_pose[4]:.2f}°, RZ={new_pose[5]:.2f}°")

    except can.CanError as e:
        print(f"CAN通信错误: {e}")
    except Exception as e:
        print(f"发生异常: {e}")
    finally:
        # 8. 程序结束前禁用机械臂:cite[1.txt]:cite[2]
        print("禁用机械臂...")
        piper.DisableArm()
        piper.DisconnectPort()
        print("程序结束，连接已断开。")

if __name__ == "__main__":
    main()