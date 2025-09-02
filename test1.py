#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
leap_to_piper_ctrl_verbose.py

控制逻辑：
- 在 piper.ConnectPort() 后立即做一次归零（JointCtrl 和 Gripper）
- 每一次 tracking cycle（start -> data...-> end）：
    * start: 记录手起始位置 hand_origin（用于增量），记录机械臂当前位置 machine_origin
    * data: 计算 delta_hand -> 目标 = machine_origin + k * delta_hand
            姿态绝对控制：quat->euler->millideg
            夹爪：pinch_strength 映射并反转
            限幅后发送 ModeCtrl + EndPoseCtrl + GripperCtrl
    * end: 立即调用 ModeCtrl(ctrl_mode=0x00) 将机械臂切到 Standby（暂停）并清理状态
- 详细调试输出
"""
import time
import math
import sys

# 参数（按需调整）
CAN_PORT = "can0"
LOOP_DT = 0.05                # 控制周期 (s)
K_POS = 1.0                   # 位置缩放
MAX_LINEAR_M_S = 0.1          # 最大线速度 (m/s)
MAX_ANGLE_DEG_S = 20.0        # 最大角速度 (deg/s)
GRIPPER_FULL_RANGE_MM = 100.0
MOVE_SPEED_PERCENT = 60       # ModeCtrl move speed (0-100)
# 转换因子
M_TO_PIPER_POS = 1e6    # m -> piper unit (0.001mm)
DEG_TO_MILLIDEG = 1000  # deg -> millideg

# 计算每周期最大步长（整数）
MAX_STEP_POS_UNITS = max(1, int(MAX_LINEAR_M_S * M_TO_PIPER_POS * LOOP_DT))
MAX_STEP_ANGLE_UNITS = max(1, int(MAX_ANGLE_DEG_S * DEG_TO_MILLIDEG * LOOP_DT))

# helper: angle normalization (millideg)
def shortest_angle_diff_md(target_md, last_md):
    diff = (int(target_md) - int(last_md) + 180000) % 360000 - 180000
    return int(diff)

def limit_step(target, last, max_step):
    if last is None:
        return int(target)
    delta = int(target) - int(last)
    if delta > max_step:
        delta = max_step
    elif delta < -max_step:
        delta = -max_step
    return int(last + delta)

def limit_angle_step(target_md, last_md, max_step_md):
    if last_md is None:
        return int(target_md)
    diff = shortest_angle_diff_md(target_md, last_md)
    if diff > max_step_md:
        diff = max_step_md
    elif diff < -max_step_md:
        diff = -max_step_md
    return int(last_md + diff)

# quat->euler (deg) (roll X, pitch Y, yaw Z)
def quat_to_euler_deg(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi/2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

# safe read of arm endpose (returns tuple ints or None)
def read_arm_endpose_safe(piper, timeout_s=1.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        try:
            data = piper.GetArmEndPoseMsgs()
        except Exception as e:
            # debug
            print(f"[{time.time():.3f}] read_arm_endpose_safe: exception reading piper: {e}")
            time.sleep(0.01)
            continue
        if not data:
            time.sleep(0.01)
            continue
        try:
            ep = data.end_pose
            X = int(round(ep.X_axis))
            Y = int(round(ep.Y_axis))
            Z = int(round(ep.Z_axis))
            RX = int(round(ep.RX_axis))
            RY = int(round(ep.RY_axis))
            RZ = int(round(ep.RZ_axis))
            return (X, Y, Z, RX, RY, RZ)
        except Exception as e:
            # try dict-like fallback
            try:
                ep = data.get("end_pose", None)
                if ep:
                    X = int(round(ep["X_axis"])); Y = int(round(ep["Y_axis"])); Z = int(round(ep["Z_axis"]))
                    RX = int(round(ep["RX_axis"])); RY = int(round(ep["RY_axis"])); RZ = int(round(ep["RZ_axis"]))
                    return (X, Y, Z, RX, RY, RZ)
            except Exception:
                print(f"[{time.time():.3f}] read_arm_endpose_safe: parse error: {e}")
            time.sleep(0.01)
    return None

# main
def main():
    print("=== leap->piper controller starting ===")
    try:
        import leap_ext
    except Exception as e:
        print("ERROR: cannot import leap_ext:", e); return
    try:
        from piper_sdk import C_PiperInterface_V2
    except Exception as e:
        print("ERROR: cannot import piper_sdk:", e); return

    # init piper
    piper = C_PiperInterface_V2(CAN_PORT)
    piper.ConnectPort()
    # wait and enable
    t0 = time.time()
    while not piper.EnablePiper():
        if time.time()-t0 > 5.0:
            print(f"[{time.time():.3f}] WARNING: EnablePiper() still false after 5s, continuing anyway")
            break
        time.sleep(0.01)
    print(f"[{time.time():.3f}] Piper ConnectPort/Enable done, performing initial zeroing now")

    # One-time zeroing immediately after ConnectPort (per your request)
    try:
        piper.ModeCtrl(0x01, 0x01, 30, 0x00)   # CAN control, MOVE P as safe default
        # Joint zeros (units = 0.001 deg). set zeros for all joints
        piper.JointCtrl(0,0,0,0,0,0)
        # Open gripper (0 value) and enable
        piper.GripperCtrl(0, 1000, 0x01, 0)
        print(f"[{time.time():.3f}] Initial zeroing sent to piper.")
    except Exception as e:
        print(f"[{time.time():.3f}] Warning: zeroing commands failed: {e}")

    # init leap ext
    leap_ext.init()

    in_tracking_cycle = False
    hand_origin_m = None
    machine_origin = None
    last_sent_pose = None

    print(f"[{time.time():.3f}] Entering event loop (Ctrl-C to exit)")

    try:
        while True:
            ev = leap_ext.next_event(5)  # poll with 5 ms timeout
            ts = time.time()
            if not ev:
                # nothing new
                time.sleep(LOOP_DT)
                continue
            # debug print raw event
            print(f"[{ts:.3f}] Received event: {ev}")

            ev_type = ev.get("event") if isinstance(ev, dict) else None

            if ev_type == "connected":
                print(f"[{ts:.3f}] Leap connected")
                continue
            if ev_type == "disconnected":
                print(f"[{ts:.3f}] Leap disconnected -> pausing arm and resetting state")
                # immediately put arm to standby
                try:
                    piper.ModeCtrl(0x00, 0x01, 0, 0x00)  # Standby
                    print(f"[{time.time():.3f}] Sent ModeCtrl(standby) on disconnect")
                except Exception as e:
                    print(f"[{time.time():.3f}] Warning: ModeCtrl standby failed: {e}")
                in_tracking_cycle = False
                hand_origin_m = None
                machine_origin = None
                last_sent_pose = None
                continue

            if ev_type == "start":
                print(f"[{ts:.3f}] Tracking START frame={ev.get('frame_id')} hand={ev.get('hand_id')}")
                in_tracking_cycle = True
                hand_origin_m = None
                machine_origin = None
                last_sent_pose = None

                # hand origin (we rely on leap_ext rel_pos but if it provides abs_pos we ignore per your note)
                # leap_ext in our latest version already returns rel_pos=0 at start; safe to set origin to zero
                hand_origin_m = (0.0, 0.0, 0.0)

                # read machine origin now (for increment)
                machine_origin = read_arm_endpose_safe(piper, timeout_s=1.0)
                if machine_origin is None:
                    print(f"[{time.time():.3f}] WARNING: cannot read machine origin; using zeros")
                    machine_origin = (0,0,0,0,0,0)
                last_sent_pose = list(machine_origin)
                print(f"[{time.time():.3f}] Machine origin read: {machine_origin}")
                # we already did initial zero on connect; do not zero again per your request
                continue

            if ev_type == "data" and in_tracking_cycle:
                # we expect rel_pos in meters in the event (leap_ext provides rel_pos)
                rel = ev.get("rel_pos")
                quat = ev.get("quat")
                pinch_strength = float(ev.get("pinch_strength", 0.0))

                if rel is None:
                    # fallback: treat as zero delta
                    dx_m, dy_m, dz_m = 0.0, 0.0, 0.0
                else:
                    dx_m, dy_m, dz_m = float(rel[0]), float(rel[1]), float(rel[2])

                # compute target position in piper units (0.001 mm)
                tgt_X = int(round(machine_origin[0] + K_POS * dx_m * M_TO_PIPER_POS))
                tgt_Y = int(round(machine_origin[1] + K_POS * dy_m * M_TO_PIPER_POS))
                tgt_Z = int(round(machine_origin[2] + K_POS * dz_m * M_TO_PIPER_POS))

                # compute absolute orientation from quat if available
                if quat and len(quat) == 4:
                    qx,qy,qz,qw = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
                    rx_deg, ry_deg, rz_deg = quat_to_euler_deg(qx,qy,qz,qw)
                else:
                    rx_deg = machine_origin[3] / DEG_TO_MILLIDEG
                    ry_deg = machine_origin[4] / DEG_TO_MILLIDEG
                    rz_deg = machine_origin[5] / DEG_TO_MILLIDEG

                tgt_RX_md = int(round(rx_deg * DEG_TO_MILLIDEG))
                tgt_RY_md = int(round(ry_deg * DEG_TO_MILLIDEG))
                tgt_RZ_md = int(round(rz_deg * DEG_TO_MILLIDEG))

                # gripper: map 0..1 to 0..FULL, then invert (you said open/closed reversed)
                grip_mm = pinch_strength * GRIPPER_FULL_RANGE_MM
                grip_mm_inverted = GRIPPER_FULL_RANGE_MM - grip_mm
                gripper_units = int(round(grip_mm_inverted * 1000.0))  # 0.001 mm unit

                # rate-limit (smooth) relative to last_sent_pose
                nx = limit_step(tgt_X, None if last_sent_pose is None else last_sent_pose[0], MAX_STEP_POS_UNITS)
                ny = limit_step(tgt_Y, None if last_sent_pose is None else last_sent_pose[1], MAX_STEP_POS_UNITS)
                nz = limit_step(tgt_Z, None if last_sent_pose is None else last_sent_pose[2], MAX_STEP_POS_UNITS)

                nrx = limit_angle_step(tgt_RX_md, None if last_sent_pose is None else last_sent_pose[3], MAX_STEP_ANGLE_UNITS)
                nry = limit_angle_step(tgt_RY_md, None if last_sent_pose is None else last_sent_pose[4], MAX_STEP_ANGLE_UNITS)
                nrz = limit_angle_step(tgt_RZ_md, None if last_sent_pose is None else last_sent_pose[5], MAX_STEP_ANGLE_UNITS)

                # clamp angles to +-180 deg (millideg)
                def clamp_angle_md(x):
                    if x > 180000: return 180000
                    if x < -180000: return -180000
                    return x
                nrx, nry, nrz = clamp_angle_md(nrx), clamp_angle_md(nry), clamp_angle_md(nrz)

                # Send to arm: set ModeCtrl then EndPoseCtrl then GripperCtrl
                try:
                    # 1) Ensure CAN control mode and MOVE L (linear)
                    piper.ModeCtrl(0x01, 0x02, MOVE_SPEED_PERCENT, 0x00)
                    # 2) Send the target pose
                    piper.EndPoseCtrl(nx, ny, nz, nrx, nry, nrz)
                    # 3) Send gripper (enable)
                    piper.GripperCtrl(abs(gripper_units), 1000, 0x01, 0)
                    last_sent_pose = [nx, ny, nz, nrx, nry, nrz]
                    print(f"[{time.time():.3f}] Sent -> EndPose: X={nx} Y={ny} Z={nz} RX={nrx} RY={nry} RZ={nrz}  Gripper={gripper_units}  (frame={ev.get('frame_id')})")
                except Exception as e:
                    print(f"[{time.time():.3f}] ERROR sending to piper: {e}")

                # debug: show rel and computed targets
                print(f"[{time.time():.3f}] dbg: rel(m)=({dx_m:.4f},{dy_m:.4f},{dz_m:.4f})  tgt(mm*1e-3units)=({tgt_X},{tgt_Y},{tgt_Z})  euler_deg=({rx_deg:.2f},{ry_deg:.2f},{rz_deg:.2f}) pinch={pinch_strength:.3f}")
                time.sleep(LOOP_DT)
                continue

            if ev_type == "end":
                print(f"[{time.time():.3f}] Tracking END received -> pausing arm immediately")
                # immediate pause: set Standby mode (ctrl_mode = 0x00)
                try:
                    piper.ModeCtrl(0x00, 0x01, 0, 0x00)
                    print(f"[{time.time():.3f}] Sent ModeCtrl(standby) to piper (arm paused).")
                except Exception as e:
                    print(f"[{time.time():.3f}] Warning: ModeCtrl standby failed: {e}")
                in_tracking_cycle = False
                hand_origin_m = None
                machine_origin = None
                last_sent_pose = None
                continue

            # unknown event - print
            print(f"[{time.time():.3f}] Unknown event type: {ev_type} -> ignoring")

    except KeyboardInterrupt:
        print("User requested stop (Ctrl-C).")
    finally:
        print("Shutting down leap_ext and leaving piper in standby.")
        try:
            leap_ext.shutdown()
        except Exception:
            pass
        try:
            piper.ModeCtrl(0x00, 0x01, 0, 0x00)
        except Exception:
            pass
        print("Exited.")

if __name__ == "__main__":
    main()
