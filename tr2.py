#!/usr/bin/env python3
# leap_to_piper.py
# Requires: piper_sdk (C_PiperInterface_V2) and leap_ext (C extension)
# Usage: python3 leap_to_piper.py

import time
import math
import sys
from typing import Optional

try:
    import leap_ext   # our C extension
except Exception as e:
    raise SystemExit("leap_ext import failed: " + str(e))

try:
    from piper_sdk import C_PiperInterface_V2
except Exception as e:
    raise SystemExit("piper_sdk import failed: " + str(e))

# ----------------- Configurable parameters -----------------
CAN_PORT = "can0"
LOOP_DT = 0.05                # control loop period (s)
K_POS = 1.0                   # scaling of hand delta -> machine delta (unitless)
MAX_LINEAR_M_S = 0.1          # max linear velocity allowed for smoothing (m/s)
MAX_ANGLE_DEG_S = 20.0        # max angular speed (deg/s) for smoothing
GRIPPER_FULL_RANGE_MM = 100.0 # gripper full open range (mm)
MOVE_SPEED_PERCENT = 80       # ModeCtrl move speed percent (0-100)
# ---------------------------------------------------------

# Units:
# - leap_ext returns X/Y/Z in micrometers (um), which equals Piper EndPose unit (0.001 mm)
# - Piper EndPoseCtrl expects X/Y/Z in 0.001 mm units (i.e. um).
# - Rotations: millideg (0.001°).

MAX_STEP_POS_UNITS = int(MAX_LINEAR_M_S * 1e6 * LOOP_DT)
MAX_STEP_ANGLE_UNITS = int(MAX_ANGLE_DEG_S * 1000 * LOOP_DT)
if MAX_STEP_POS_UNITS < 1:
    MAX_STEP_POS_UNITS = 1
if MAX_STEP_ANGLE_UNITS < 1:
    MAX_STEP_ANGLE_UNITS = 1

def limit_step(target: int, last: Optional[int], max_step: int) -> int:
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

def limit_angle_step(target_md: int, last_md: Optional[int], max_step_md: int) -> int:
    if last_md is None:
        return int(target_md)
    diff = shortest_angle_diff_md(int(target_md), int(last_md))
    if diff > max_step_md:
        diff = max_step_md
    elif diff < -max_step_md:
        diff = -max_step_md
    return int(last_md + diff)

def read_arm_endpose_safe(piper, timeout_s=1.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        try:
            data = piper.GetArmEndPoseMsgs()
        except Exception:
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
        except Exception:
            try:
                ep = data.get("end_pose", None)
                if ep:
                    X = int(round(ep["X_axis"]))
                    Y = int(round(ep["Y_axis"]))
                    Z = int(round(ep["Z_axis"]))
                    RX = int(round(ep["RX_axis"]))
                    RY = int(round(ep["RY_axis"]))
                    RZ = int(round(ep["RZ_axis"]))
                    return (X, Y, Z, RX, RY, RZ)
            except Exception:
                pass
        time.sleep(0.01)
    return None

def main():
    print("Starting Piper interface on", CAN_PORT)
    piper = C_PiperInterface_V2(CAN_PORT)
    piper.ConnectPort()

    # Immediately do zeroing AFTER ConnectPort and (attempted) EnablePiper:
    t0 = time.time()
    while not piper.EnablePiper():
        if time.time() - t0 > 5.0:
            print("Warning: EnablePiper() still False after 5s, continuing anyway...")
            break
        time.sleep(0.01)

    # One-time zero: joint zero + open gripper (this runs immediately after connecting)
    print("Performing initial ONE-TIME zero (joint zeros + gripper open)...")
    try:
        piper.ModeCtrl(0x01, 0x01, 30, 0x00)  # CAN control, MOVE P
    except Exception:
        pass
    try:
        piper.JointCtrl(0,0,0,0,0,0)
    except Exception:
        pass
    try:
        piper.GripperCtrl(0, 1000, 0x01, 0)
    except Exception:
        pass

    # Initialize Leap
    print("Initializing Leap connection...")
    leap_ext.init()

    in_tracking = False
    machine_origin = None
    last_sent = None

    print("Ready. Waiting for leap events... (Ctrl-C to stop)")
    try:
        while True:
            ev = leap_ext.next_event()
            if not ev:
                time.sleep(LOOP_DT)
                continue

            ev_type = ev.get("event") if isinstance(ev, dict) else None

            if ev_type == "connected":
                print("[leap] connected")
                continue
            if ev_type == "disconnected":
                print("[leap] disconnected")
                in_tracking = False
                machine_origin = None
                last_sent = None
                continue

            # start: record machine_origin (one per tracking run). DO NOT zero here.
            if ev_type == "start":
                print("[leap] tracking START frame=", ev.get("frame_id"))
                in_tracking = True
                machine_origin = read_arm_endpose_safe(piper, timeout_s=1.0)
                if machine_origin is None:
                    print("Warning: cannot read machine endpose; assuming zeros")
                    machine_origin = (0,0,0,0,0,0)
                last_sent = list(machine_origin)
                continue

            if ev_type == "data" and in_tracking:
                # required values: relative deltas in um and rotations millideg
                X_um = int(ev.get("X", 0))
                Y_um = int(ev.get("Y", 0))
                Z_um = int(ev.get("Z", 0))
                RX_md = int(ev.get("RX", 0))
                RY_md = int(ev.get("RY", 0))
                RZ_md = int(ev.get("RZ", 0))
                pinch = float(ev.get("pinch_strength", 0.0))

                if machine_origin is None:
                    machine_origin = read_arm_endpose_safe(piper, timeout_s=0.5) or (0,0,0,0,0,0)
                    last_sent = list(machine_origin)

                tgt_X = int(round(machine_origin[0] + K_POS * X_um))
                tgt_Y = int(round(machine_origin[1] + K_POS * Y_um))
                tgt_Z = int(round(machine_origin[2] + K_POS * Z_um))
                tgt_RX = int(RX_md)
                tgt_RY = int(RY_md)
                tgt_RZ = int(RZ_md)

                # gripper invert mapping
                grip_mm = pinch * GRIPPER_FULL_RANGE_MM
                grip_mm_inverted = GRIPPER_FULL_RANGE_MM - grip_mm
                gripper_units = int(round(grip_mm_inverted * 1000.0))  # 0.001 mm

                nx = limit_step(tgt_X, None if last_sent is None else last_sent[0], MAX_STEP_POS_UNITS)
                ny = limit_step(tgt_Y, None if last_sent is None else last_sent[1], MAX_STEP_POS_UNITS)
                nz = limit_step(tgt_Z, None if last_sent is None else last_sent[2], MAX_STEP_POS_UNITS)

                nrx = limit_angle_step(tgt_RX, None if last_sent is None else last_sent[3], MAX_STEP_ANGLE_UNITS)
                nry = limit_angle_step(tgt_RY, None if last_sent is None else last_sent[4], MAX_STEP_ANGLE_UNITS)
                nrz = limit_angle_step(tgt_RZ, None if last_sent is None else last_sent[5], MAX_STEP_ANGLE_UNITS)

                def clamp_md(v):
                    if v > 180000: return 180000
                    if v < -180000: return -180000
                    return v
                nrx = clamp_md(nrx); nry = clamp_md(nry); nrz = clamp_md(nrz)

                try:
                    piper.ModeCtrl(0x01, 0x02, MOVE_SPEED_PERCENT, 0x00)  # CAN control, MOVE L
                    piper.EndPoseCtrl(nx, ny, nz, nrx, nry, nrz)
                    piper.GripperCtrl(abs(gripper_units), 1000, 0x01, 0)
                    last_sent = [nx, ny, nz, nrx, nry, nrz]
                except Exception as e:
                    print("Warning: send to piper failed:", e)

                print(f"[data] frame={ev.get('frame_id')} hand={ev.get('hand_id')}")
                print(f"       rel (um): X={X_um} Y={Y_um} Z={Z_um}")
                print(f"       rot (millideg): RX={nrx} RY={nry} RZ={nrz}")
                print(f"       -> send: X={nx} Y={ny} Z={nz} RX={nrx} RY={nry} RZ={nrz} gripper={gripper_units}")
                time.sleep(LOOP_DT)
                continue

            if ev_type == "end":
                print("[leap] tracking END")
                in_tracking = False
                machine_origin = None
                last_sent = None
                continue

    except KeyboardInterrupt:
        print("KeyboardInterrupt - exiting")
    finally:
        try:
            leap_ext.shutdown()
        except Exception:
            pass
        print("Exiting.")

if __name__ == "__main__":
    main()
