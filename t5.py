"""
Test script for Piper SDK V2:
 - connect
 - enable arm
 - ReqMasterArmMoveToHome(1.txt)
 - move linear (MoveL) +200mm in Z relative to current pose
 - wait until reached
"""

import time
import math
import sys
from pprint import pprint

# Try importing common entrypoints from piper_sdk; adapt if your package exposes different names.
try:
    # Most installs expose top-level convenience classes
    from piper_sdk import C_PiperInterface_V2 as PiperClass
except Exception:
    try:
        from piper_sdk import C_PiperInterface as PiperClass  # fallback
    except Exception:
        # try deeper import (older/newer packaging)
        try:
            from piper_sdk.interface.piper_interface import C_PiperInterface as PiperClass
        except Exception:
            PiperClass = None

if PiperClass is None:
    print("无法找到 C_PiperInterface_V2 或 C_PiperInterface，请确认 piper_sdk 已正确安装且包含 V2 类。")
    sys.exit(1)


# ---------- unit helpers ----------
def mm_to_unit_0_001mm(mm: float) -> int:
    """convert mm -> units of 0.001 mm (doc: X/Y/Z uses 0.001 mm unit)"""
    return int(round(mm * 1000.0))


def unit_0_001mm_to_mm(u: int) -> float:
    return u / 1000.0


def deg_to_unit_0_001deg(deg: float) -> int:
    """degrees -> units of 0.001 degree"""
    return int(round(deg * 1000.0))


def unit_0_001deg_to_deg(u: int) -> float:
    return u / 1000.0


def mps_to_unit_0_001ms(v_m_s: float) -> int:
    """m/s -> unit 0.001 m/s"""
    return int(round(v_m_s / 0.001))


# ---------- main logic ----------
def main():
    can_port = "can0"
    print("使用 Piper SDK 类：", PiperClass)

    # Create instance (use recommended parameters; adjust logger_level if you want more logs)
    piper = PiperClass(
        can_name=can_port,
        judge_flag=True,
        can_auto_init=True,
        dh_is_offset=1,
        start_sdk_joint_limit=False,
        start_sdk_gripper_limit=False,
        logger_level=None,  # keep default or set LogLevel.INFO
    )

    # Connect / start read thread / initialization
    print("连接 CAN 并启动 SDK 线程...")
    piper.ConnectPort(can_init=True, piper_init=True, start_thread=True)
    time.sleep(0.1)

    # check connection working
    if not piper.get_connect_status():
        print("警告：ConnectPort 调用后 get_connect_status() 返回 False，继续但请检查 CAN/线缆/权限。")

    # enable all arm motors (7 = all)
    print("使能电机（启用）...")
    try:
        piper.EnableArm(motor_num=7, enable_flag=0x02)
    except AttributeError:
        print("EnableArm 方法未找到，请检查 SDK 版本。")

    time.sleep(0.5)

    # Request home (主臂归零)
    print("请求主臂归零 (ReqMasterArmMoveToHome(1.txt)) ...")
    try:
        piper.ReqMasterArmMoveToHome(1)  # mode 1.txt: 主臂归零
    except AttributeError:
        print("ReqMasterArmMoveToHome 未实现，请使用你的 SDK 的归零函数名。")
    # 等待一段时间让归零命令执行（实际时间根据机械臂固件/运动速度调整）
    timeout_home = 30.0
    t0 = time.time()
    while time.time() - t0 < timeout_home:
        # 尝试读末端位姿或关节反馈以判断是否归零完成
        try:
            pose = piper.GetArmEndPoseMsgs()  # returns list [X,Y,Z,RX,RY,RZ] in 0.001mm / 0.001deg
        except Exception:
            pose = None

        if pose:
            # 如果 pose 非零（有反馈），我们认为它已初始化/在反馈
            print("home 过程反馈末端 pose（单位 0.001 mm / 0.001 deg）：", pose)
            break
        time.sleep(0.3)
    else:
        print("警告：等待归零超时（%fs）。继续执行后续步骤，但请在真实任务中确认是否归零成功。" % timeout_home)

    time.sleep(1.0)

    # 获取当前末端姿态
    try:
        current_pose = piper.GetArmEndPoseMsgs()
    except Exception:
        current_pose = None

    if not current_pose:
        # 没有反馈时，用默认/0并提示
        print("未获取到当前末端姿态的反馈（GetArmEndPoseMsgs 返回 None 或未实现）。请检查 SDK 配置/是否启用了定期反馈（ArmParamEnquiryAndConfig/data_feedback_0x48x）")
        # 仍然继续，但需要小心
        current_pose = [0, 0, 0, 0, 0, 0]

    # current_pose 单位：0.001 mm / 0.001 deg
    curX, curY, curZ, curRX, curRY, curRZ = current_pose
    print("当前末端（单位 0.001 mm / 0.001 deg):", current_pose)
    print("当前末端（mm / deg): X=%.3f mm Y=%.3f mm Z=%.3f mm RX=%.3f° RY=%.3f° RZ=%.3f°" % (
        unit_0_001mm_to_mm(curX), unit_0_001mm_to_mm(curY), unit_0_001mm_to_mm(curZ),
        unit_0_001deg_to_deg(curRX), unit_0_001deg_to_deg(curRY), unit_0_001deg_to_deg(curRZ)
    ))

    # 目标：Z + 200 mm（20cm）
    delta_z_mm = 200.0
    targetX = curX
    targetY = curY
    targetZ = curZ + mm_to_unit_0_001mm(delta_z_mm)
    targetRX = curRX
    targetRY = curRY
    targetRZ = curRZ

    print("目标末端（单位 0.001 mm / 0.001 deg):", [targetX, targetY, targetZ, targetRX, targetRY, targetRZ])
    print("目标末端（mm / deg): X=%.3f mm Y=%.3f mm Z=%.3f mm" % (
        unit_0_001mm_to_mm(targetX), unit_0_001mm_to_mm(targetY), unit_0_001mm_to_mm(targetZ)
    ))

    # 设置末端速度/加速度参数（EndSpdAndAccParamSet）
    # 例如：线速度 0.2 m/s -> 单位 0.001 m/s = 200
    try:
        max_linear_vel_unit = mps_to_unit_0_001ms(0.2)  # 0.2 m/s
        piper.EndSpdAndAccParamSet(
            max_linear_vel=max_linear_vel_unit,
            max_angular_vel=1000,       # 单位 0.001 rad/s (示意)
            max_linear_acc=500,         # 单位 0.001 m/s^2
            max_angular_acc=500         # 单位 0.001 rad/s^2
        )
        print("已设置末端速度/加速参数")
    except AttributeError:
        print("EndSpdAndAccParamSet 未实现或命名不同，请参照 INTERFACE_V2.MD 调整。")

    # 切换模式为 Move L（线性运动），并把 ctrl_mode 设为 CAN 控制
    try:
        piper.ModeCtrl(ctrl_mode=0x01, 移动模式=0x02, move_spd_rate_ctrl=50, is_mit_mode=0x00)
        # NOTE: above call uses Chinese-keyword name from doc snippet; if your SDK has english param names,
        # replace by: piper.ModeCtrl(ctrl_mode=0x01, move_mode=0x02, move_spd_rate_ctrl=50, is_mit_mode=0x00)
        print("已设置运动模式：Move L（线性）")
    except TypeError:
        # try english param names fallback
        try:
            piper.ModeCtrl(ctrl_mode=0x01, move_mode=0x02, move_spd_rate_ctrl=50, is_mit_mode=0x00)
            print("已设置运动模式：Move L（线性） (english param names)")
        except Exception as e:
            print("设置运动模式失败：", e)
            print("请自行用正确的 ModeCtrl 参数名（参照你的 SDK）")

    time.sleep(0.1)

    # 发送末端目标（EndPoseCtrl），单位：0.001 mm / 0.001 deg
    try:
        piper.EndPoseCtrl(int(targetX), int(targetY), int(targetZ), int(targetRX), int(targetRY), int(targetRZ))
        print("发送 EndPoseCtrl (目标位姿)")
    except Exception as e:
        print("发送 EndPoseCtrl 失败：", e)
        print("可能需要先调用 ModeCtrl 设置为 L 模式，或 SDK 的 EndPoseCtrl 名称/参数顺序不同。")

    # 等待并轮询是否达成目标
    tol_mm = 5.0  # 容差 5 mm
    tol_units = mm_to_unit_0_001mm(tol_mm)
    wait_timeout = 30.0
    tstart = time.time()
    reached = False
    while time.time() - tstart < wait_timeout:
        time.sleep(0.2)
        try:
            fb = piper.GetArmEndPoseMsgs()
        except Exception:
            fb = None
        if not fb:
            print("未读取到末端反馈帧，继续轮询...")
            continue
        # fb units: 0.001 mm / 0.001 deg
        fx, fy, fz, frx, fry, frz = fb
        dx = abs(fx - targetX)
        dy = abs(fy - targetY)
        dz = abs(fz - targetZ)
        # print short live feedback
        print(f"反馈 Z={unit_0_001mm_to_mm(fz):.2f}mm, 误差 dz={unit_0_001mm_to_mm(dz):.2f}mm")
        if dx <= tol_units and dy <= tol_units and dz <= tol_units:
            reached = True
            break

    if reached:
        print("目标位姿已到达（在容差范围内）。")
    else:
        print("超时：未在指定时间内到达目标位姿，请检查状态/参数/碰撞保护等。")

    # 可选：停机/卸能
    try:
        print("禁用臂电机（DisableArm）...")
        piper.DisableArm(motor_num=7, enable_flag=0x01)
    except Exception:
        pass

    print("测试结束。")

if __name__ == "__main__":
    main()