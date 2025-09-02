#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
piper_orientation_test.py
测试：保持 XYZ 不动，仅用 LeapExt 输出的姿态控制末端的 RX/RY/RZ。
"""

import time
import sys

# config
CAN_PORT = "can0"
MOVE_SPEED_PERCENT = 30
STEP_DT = 0.3
DEG_TO_MILLIDEG = 1000  # 1° = 1000 millideg

import leap_ext  # C 扩展模块

TEST_ORIENTATIONS_DEG = [
    (0.0, 0.0, 0.0),
    (20.0, 0.0, 0.0),
    (-20.0, 0.0, 0.0),
    (0.0, 15.0, 0.0),
    (0.0, -15.0, 0.0),
    (0.0, 0.0, 30.0),
    (0.0, 0.0, -30.0),
    (0.0, 0.0, 0.0),
]


def read_arm_endpose_safe(piper, timeout_s=1.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        try:
            data = piper.GetArmEndPoseMsgs()
        except Exception as e:
            print(f"[read_arm] exception reading: {e}")
            time.sleep(0.05)
            continue
        if not data:
            time.sleep(0.02)
            continue
        try:
            ep = data.end_pose
            return (
                int(round(ep.X_axis)),
                int(round(ep.Y_axis)),
                int(round(ep.Z_axis)),
                int(round(ep.RX_axis)),
                int(round(ep.RY_axis)),
                int(round(ep.RZ_axis)),
            )
        except Exception as e:
            print("[read_arm] parse failed:", e)
        time.sleep(0.02)
    return None


def main():
    print("=== Piper orientation-only test (with LeapExt debug) ===")
    try:
        from piper_sdk import C_PiperInterface_V2
    except Exception as e:
        print("ERROR: cannot import piper_sdk:", e)
        return

    piper = C_PiperInterface_V2(CAN_PORT)
    piper.ConnectPort()

    tstart = time.time()
    while not piper.EnablePiper():
        if time.time() - tstart > 5.0:
            print("Warning: EnablePiper still False after 5s")
            break
        time.sleep(0.02)

    # 初始归零
    try:
        print("Zeroing joints and gripper...")
        piper.ModeCtrl(0x01, 0x01, 20, 0x00)
        time.sleep(0.05)
        piper.JointCtrl(0, 0, 0, 0, 0, 0)
        time.sleep(0.05)
        piper.GripperCtrl(0, 1000, 0x01, 0)
    except Exception as e:
        print("Zeroing failed:", e)

    base = read_arm_endpose_safe(piper, timeout_s=2.0)
    if base is None:
        print("ERROR: cannot read base end-pose; abort.")
        return
    base_X, base_Y, base_Z, _, _, _ = base
    print(f"Base fixed position: X={base_X}, Y={base_Y}, Z={base_Z}")

    try:
        for idx, (rx_deg, ry_deg, rz_deg) in enumerate(TEST_ORIENTATIONS_DEG):
            tgt_RX = int(round(rx_deg * DEG_TO_MILLIDEG))
            tgt_RY = int(round(ry_deg * DEG_TO_MILLIDEG))
            tgt_RZ = int(round(rz_deg * DEG_TO_MILLIDEG))

            # 调试输出 LeapExt 的最新数据
            try:
                leap_pose = leap_ext.get_latest_pose()
                print(f"[LeapExt] {leap_pose}")
            except Exception as e:
                print("[LeapExt] get_latest_pose failed:", e)

            print(f"[{idx}] Send RX={rx_deg}° RY={ry_deg}° RZ={rz_deg}° "
                  f"-> ({tgt_RX},{tgt_RY},{tgt_RZ}) millideg")
            try:
                piper.ModeCtrl(0x01, 0x02, MOVE_SPEED_PERCENT, 0x00)
                piper.EndPoseCtrl(base_X, base_Y, base_Z, tgt_RX, tgt_RY, tgt_RZ)
            except Exception as e:
                print("ERROR EndPoseCtrl:", e)

            time.sleep(STEP_DT)
            cur = read_arm_endpose_safe(piper, timeout_s=0.8)
            print("  read-back:", cur)
        print("Finished orientation sequence.")
    except KeyboardInterrupt:
        print("User interrupted.")

    try:
        piper.ModeCtrl(0x00, 0x01, 0, 0x00)
        print("Standby mode set.")
    except Exception as e:
        print("Standby failed:", e)


if __name__ == "__main__":
    main()
