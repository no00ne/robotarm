#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
leap_to_piper_mock.py

Test harness: reads events from leap_ext and prints the commands that would be sent to Piper.
No real Piper SDK is called.

Expected leap_ext event format (dict):
 - "event": "connected" | "disconnected" | "start" | "data" | "end"
 - For "start": fields "frame_id","hand_id","X","Y","Z","RX","RY","RZ"
   (X/Y/Z are relative in μm; RX/RY/RZ are absolute millideg)
 - For "data": fields "frame_id","hand_id","X","Y","Z","RX","RY","RZ","pinch_strength"
"""
import sys
import time
import math

# ---------- configuration ----------
LOOP_DT = 0.05                # main loop sleep (s)
K_POS = 1.0                   # scaling: machine_target = machine_origin + K_POS * hand_rel_um
GRIPPER_FULL_RANGE_MM = 100.0 # gripper physical max (mm)
MOVE_SPEED_PERCENT = 80
MAX_LINEAR_M_S = 0.1          # for step limiting (optional)
MAX_ANGLE_DEG_S = 20.0
TIMEOUT_MS=10
# -----------------------------------

# unit helpers
DEG_TO_MILLIDEG = 1000
MAX_STEP_POS_UNITS = max(1, int(MAX_LINEAR_M_S * 1e6 * LOOP_DT))        # μm per cycle
MAX_STEP_ANGLE_UNITS = max(1, int(MAX_ANGLE_DEG_S * DEG_TO_MILLIDEG * LOOP_DT))

def limit_step(target: int, last: int or None, max_step: int) -> int:
    if last is None:
        return int(target)
    delta = int(target) - int(last)
    if delta > max_step:
        delta = max_step
    elif delta < -max_step:
        delta = -max_step
    return int(last + delta)

def shortest_angle_diff_md(target_md: int, last_md: int) -> int:
    diff = (target_md - last_md + 180000) % 360000 - 180000
    return int(diff)

def limit_angle_step(target_md: int, last_md: int or None, max_step_md: int) -> int:
    if last_md is None:
        return int(target_md)
    diff = shortest_angle_diff_md(int(target_md), int(last_md))
    if diff > max_step_md:
        diff = max_step_md
    elif diff < -max_step_md:
        diff = -max_step_md
    return int(last_md + diff)

def clamp_md(x: int) -> int:
    if x > 180000: return 180000
    if x < -180000: return -180000
    return x

# ---------- main test harness ----------
def main():
    try:
        import leap_ext
    except Exception as e:
        print("ERROR: cannot import leap_ext. Build and install the C extension first. Exception:", e)
        return

    print("[MOCK] Starting test harness (no piper calls).")
    print("[MOCK] Immediate 'zero' action performed (simulated). Machine origin will be read as zeros.")
    print("初始化 leap_ext...")
    try:
        # 如果你的 py 扩展接收参数，比如 optimize_hmd，可以传入 True/False
        leap_ext.init()
    except Exception as e:
        print("leap_ext.init() 报错：", e)
        sys.exit(1)
    # simulate "machine origin" read at connect/zero: since no real robot, use zeros
    machine_origin = (0,0,0, 0,0,0)   # units: X/Y/Z in μm, RX/RY/RZ in millideg
    last_sent_pose = list(machine_origin)

    in_tracking_cycle = False

    try:
        while True:
            # call leap_ext.next_event with short timeout if supported
            ev = None
            try:
                ev = leap_ext.next_event(TIMEOUT_MS)
            except TypeError:
                # fallback if binding doesn't accept args
                ev = leap_ext.next_event()
            except Exception as e:
                print("[MOCK] leap_ext.next_event() exception:", e)
                time.sleep(LOOP_DT)
                continue

            if ev is None:
                time.sleep(LOOP_DT)
                continue

            if not isinstance(ev, dict):
                print("[MOCK] unexpected event type:", type(ev), ev)
                time.sleep(LOOP_DT)
                continue

            ev_type = ev.get("event")
            if ev_type == "connected":
                print("[LEAP] connected")
                # simulated zero already done at startup
                continue

            if ev_type == "disconnected":
                print("[LEAP] disconnected -> would send ModeCtrl(standby) and clear state")
                in_tracking_cycle = False
                last_sent_pose = None
                continue

            if ev_type == "start":
                print(f"[LEAP] START frame={ev.get('frame_id')} hand={ev.get('hand_id')}")
                in_tracking_cycle = True
                # record machine_origin once for this tracking cycle (we simulate 0)
                machine_origin = (0,0,0, 0,0,0)
                last_sent_pose = list(machine_origin)
                print("[MOCK] machine_origin set to", machine_origin)
                continue

            if ev_type == "data" and in_tracking_cycle:
                # parse expected fields (C extension provides X/Y/Z in μm and RX/RY/RZ in millideg)
                try:
                    X_um = int(ev.get("X", 0))
                    Y_um = int(ev.get("Y", 0))
                    Z_um = int(ev.get("Z", 0))
                    RX_md = int(ev.get("RX", 0))-180000
                    RY_md = int(ev.get("RY", 0))
                    RZ_md = int(ev.get("RZ", 0))-180000
                    pinch_strength = float(ev.get("pinch_strength", 0.0))
                except Exception as e:
                    print("[MOCK] bad data frame format:", e, "frame:", ev)
                    time.sleep(LOOP_DT)
                    continue

                # compute target (incremental control)
                tgt_X = int(round(machine_origin[0] + K_POS * X_um))
                tgt_Y = int(round(machine_origin[1] + K_POS * Y_um))
                tgt_Z = int(round(machine_origin[2] + K_POS * Z_um))
                tgt_RX = int(round(RX_md))
                tgt_RY = int(round(RY_md))
                tgt_RZ = int(round(RZ_md))

                # gripper: pinch_strength 0..1 -> 0..GRIPPER_FULL_RANGE_MM (mm) then invert
                grip_mm = pinch_strength * GRIPPER_FULL_RANGE_MM
                grip_mm_inv = GRIPPER_FULL_RANGE_MM - grip_mm
                gripper_units = int(round(grip_mm_inv * 1000.0))  # to 0.001 mm

                # smoothing/limiting from last_sent_pose
                nx = limit_step(tgt_X, None if last_sent_pose is None else last_sent_pose[0], MAX_STEP_POS_UNITS)
                ny = limit_step(tgt_Y, None if last_sent_pose is None else last_sent_pose[1], MAX_STEP_POS_UNITS)
                nz = limit_step(tgt_Z, None if last_sent_pose is None else last_sent_pose[2], MAX_STEP_POS_UNITS)

                nrx = limit_angle_step(tgt_RX, None if last_sent_pose is None else last_sent_pose[3], MAX_STEP_ANGLE_UNITS)
                nry = limit_angle_step(tgt_RY, None if last_sent_pose is None else last_sent_pose[4], MAX_STEP_ANGLE_UNITS)
                nrz = limit_angle_step(tgt_RZ, None if last_sent_pose is None else last_sent_pose[5], MAX_STEP_ANGLE_UNITS)

                nrx = clamp_md(nrx); nry = clamp_md(nry); nrz = clamp_md(nrz)

                # PRINT what would be sent to Piper
                print("-----------------------------------------------------------")
                print(f"[DATA] frame={ev.get('frame_id')} hand={ev.get('hand_id')}")
                print(f"  hand rel (μm): X={X_um} Y={Y_um} Z={Z_um}")
                print(f"  computed target (pre-limit): X={tgt_X} Y={tgt_Y} Z={tgt_Z}  RX={tgt_RX} RY={tgt_RY} RZ={tgt_RZ}")
                print(f"  after limiting -> EndPoseCtrl(X,Y,Z,RX,RY,RZ) = ({nx},{ny},{nz},{nrx},{nry},{nrz})")
                print(f"  pinch_strength={pinch_strength:.3f} -> gripper_units (0.001mm) = {gripper_units} (inverted)")
                print(f"  Would call: ModeCtrl(ctrl=0x01, move=0x02, speed={MOVE_SPEED_PERCENT})")
                print(f"            EndPoseCtrl({nx},{ny},{nz},{nrx},{nry},{nrz})")
                print(f"            GripperCtrl({abs(gripper_units)}, effort=1000, code=0x01, set_zero=0)")
                print("-----------------------------------------------------------")

                last_sent_pose = [nx, ny, nz, nrx, nry, nrz]

                time.sleep(LOOP_DT)
                continue

            if ev_type == "end":
                print("[LEAP] END -> would send ModeCtrl(standby) and clear tracking state")
                in_tracking_cycle = False
                last_sent_pose = None
                continue

            # unknown event
            print("[MOCK] unknown event:", ev)
            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        print("User interrupted. Exiting.")
    finally:
        try:
            leap_ext.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
