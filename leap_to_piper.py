#!/usr/bin/env python3
# leap_to_piper.py
# Controls Piper using leap_ext (C extension).
# Position: incremental control; Orientation: absolute; Gripper: absolute (mapped & inverted)
# Prints C-frame-rate and Python processing fps once per second.

import time
import math
import sys

# Config
CAN_PORT = "can0"
LOOP_DT = 0.05                # control loop (s) - sleep between cycles
K_POS = 1.0                   # scale for position mapping (X_machine = X_machine0 + K_POS * delta_hand)
MAX_LINEAR_M_S = 0.1          # not used directly if units are micrometers; used to compute smoothing
MAX_ANGLE_DEG_S = 20.0        # deg/s for smoothing
GRIPPER_FULL_RANGE_MM = 100.0 # physical gripper range in mm
MOVE_SPEED_PERCENT = 80

# Derived smoothing limits per loop (units matching Piper: position in piper units (1 unit = 0.001mm = 1um),
# angle in millideg (0.001 deg))
MAX_STEP_POS_UNITS = max(1, int(MAX_LINEAR_M_S * 1e6 * LOOP_DT))       # meters->piper units: m*1e6, but MAX_LINEAR_M_S is m/s
MAX_STEP_ANGLE_UNITS = max(1, int(MAX_ANGLE_DEG_S * 1000.0 * LOOP_DT)) # deg->millideg

# Utilities: smoothing and angle wrap
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

# Safe read of arm end pose (returns tuple of integers in piper units (X,Y,Z,RX,RY,RZ) or None)
def read_arm_endpose_safe(piper, timeout_s=0.8):
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
            # try dict-like fallback
            try:
                ep = data.get("end_pose", None)
                if ep:
                    X = int(round(ep["X_axis"])); Y = int(round(ep["Y_axis"])); Z = int(round(ep["Z_axis"]))
                    RX = int(round(ep["RX_axis"])); RY = int(round(ep["RY_axis"])); RZ = int(round(ep["RZ_axis"]))
                    return (X, Y, Z, RX, RY, RZ)
            except Exception:
                pass
        time.sleep(0.01)
    return None

def map_gripper_units_from_pinch(pinch_strength: float) -> int:
    # pinch_strength in [0,1]. Map to 0..GRIPPER_FULL_RANGE_MM (mm). Invert because user said open/close reversed.
    mm = pinch_strength * GRIPPER_FULL_RANGE_MM
    inv_mm = GRIPPER_FULL_RANGE_MM - mm
    units = int(round(inv_mm * 1000.0))  # piper expects 0.001mm units
    return max(0, units)

def main():
    try:
        import leap_ext
    except Exception as e:
        print("ERROR: leap_ext not available:", e)
        sys.exit(1)

    try:
        from piper_sdk import C_PiperInterface_V2
    except Exception as e:
        print("ERROR: piper_sdk not available:", e)
        sys.exit(1)

    # init piper
    print("[main] Creating Piper interface on", CAN_PORT)
    piper = C_PiperInterface_V2(CAN_PORT)
    piper.ConnectPort()
    # Wait for enable
    tstart = time.time()
    while not piper.EnablePiper():
        if time.time() - tstart > 5.0:
            print("[main] Warning: EnablePiper() still False after 5s, continuing anyway")
            break
        time.sleep(0.01)

    # Immediately zero once (as requested)
    print("[main] Zeroing robot now (one-time after ConnectPort())")
    try:
        piper.ModeCtrl(0x01, 0x01, 30, 0x00)  # CAN control, move mode P
        piper.JointCtrl(0, 0, 0, 0, 0, 0)
        piper.GripperCtrl(0, 1000, 0x01, 0)   # enable gripper position 0
    except Exception as e:
        print("[main] Warning: zeroing commands raised:", e)

    # init leap
    try:
        leap_ext.init()
    except Exception as e:
        print("[main] leap_ext.init() failed:", e)
        # continue (may still work if extension already initialized)
    print("[main] Initialization done. Entering loop.")

    # state
    in_tracking_cycle = False
    hand_origin = None           # not needed beyond starting event (we use rel provided by C)
    machine_origin = None        # machine origin: tuple of piper units (X,Y,Z,RX,RY,RZ)
    last_sent_pose = None        # last sent [X,Y,Z,RX,RY,RZ] in piper units
    zero_performed_after_connect = True  # we already zeroed once

    # fps stats
    py_frames_processed = 0      # number of data events processed by python in current second
    c_frame_ids_seen = set()     # unique frame ids observed this second
    last_stats_time = time.time()
    last_frame_id_seen = None    # for delta-based C fps if desired

    try:
        while True:
            # request next event with small timeout to allow loop timing
            ev = leap_ext.next_event(10)  # ms
            tnow = time.time()

            if ev is None:
                # no event; we still print stats periodically
                # sleep a bit to enforce loop rate
                time.sleep(LOOP_DT)
            else:
                ev_type = ev.get("event") if isinstance(ev, dict) else None

                # connected / disconnected
                if ev_type == "connected":
                    print("[leap] connected")
                    # nothing else
                elif ev_type == "disconnected":
                    print("[leap] disconnected - clearing state")
                    in_tracking_cycle = False
                    hand_origin = None
                    machine_origin = None
                    last_sent_pose = None
                elif ev_type == "start":
                    print(f"[leap] START frame={ev.get('frame_id')} hand={ev.get('hand_id')}")
                    in_tracking_cycle = True
                    # record machine origin now (read once)
                    mo = read_arm_endpose_safe(piper, timeout_s=1.0)
                    if mo is None:
                        print("[main] Warning: cannot read machine endpose; assuming zeros")
                        mo = (0,0,0,0,0,0)
                    machine_origin = mo
                    last_sent_pose = list(machine_origin)
                    # start stats reset for this cycle
                    py_frames_processed = 0
                    c_frame_ids_seen.clear()
                    # we do not send motion at start (pos=0 rel). continue to next events.
                elif ev_type == "data" and in_tracking_cycle:
                    # data contains X/Y/Z (relative in μm), RX/RY/RZ (millideg), pinch_strength
                    frame_id = ev.get("frame_id")
                    c_frame_ids_seen.add(frame_id)
                    py_frames_processed += 1

                    # read relative deltas in piper units (they are μm => same unit as piper pos)
                    dx = int(ev.get("X", 0))
                    dy = int(ev.get("Y", 0))
                    dz = int(ev.get("Z", 0))
                    rx = int(ev.get("RX", 0))-90000
                    ry = int(ev.get("RY", 0))
                    rz = int(ev.get("RZ", 0))
                    pinch = float(ev.get("pinch_strength", 0.0))

                    # compute targets
                    # machine_origin is in piper units already
                    if machine_origin is None:
                        # fallback read
                        mo = read_arm_endpose_safe(piper, timeout_s=0.5)
                        if mo is None:
                            machine_origin = (0,0,0,0,0,0)
                        else:
                            machine_origin = mo
                            if last_sent_pose is None:
                                last_sent_pose = list(machine_origin)

                    tgt_X = int(round(machine_origin[0] + K_POS * dx))
                    tgt_Y = int(round(machine_origin[1] + K_POS * dy))
                    tgt_Z = int(round(machine_origin[2] + K_POS * dz))

                    tgt_RX = int(round(rx))
                    tgt_RY = int(round(ry))
                    tgt_RZ = int(round(rz))

                    # gripper
                    gripper_units = map_gripper_units_from_pinch(pinch)

                    # smoothing / limit step
                    nx = limit_step(tgt_X, None if last_sent_pose is None else last_sent_pose[0], MAX_STEP_POS_UNITS)
                    ny = limit_step(tgt_Y, None if last_sent_pose is None else last_sent_pose[1], MAX_STEP_POS_UNITS)
                    nz = limit_step(tgt_Z, None if last_sent_pose is None else last_sent_pose[2], MAX_STEP_POS_UNITS)

                    nrx = limit_angle_step(tgt_RX, None if last_sent_pose is None else last_sent_pose[3], MAX_STEP_ANGLE_UNITS)
                    nry = limit_angle_step(tgt_RY, None if last_sent_pose is None else last_sent_pose[4], MAX_STEP_ANGLE_UNITS)
                    nrz = limit_angle_step(tgt_RZ, None if last_sent_pose is None else last_sent_pose[5], MAX_STEP_ANGLE_UNITS)

                    # clamp angles to reasonable bounds
                    def clamp_angle_md(x):
                        if x > 180000:
                            return 180000
                        if x < -180000:
                            return -180000
                        return x
                    nrx = clamp_angle_md(nrx); nry = clamp_angle_md(nry); nrz = clamp_angle_md(nrz)

                    # send to arm
                    try:
                        piper.ModeCtrl(0x01, 0x02, MOVE_SPEED_PERCENT, 0x00)  # CAN control, move L (linear)
                        piper.EndPoseCtrl(nx, ny, nz, nrx, nry, nrz)
                        piper.GripperCtrl(abs(gripper_units), 1000, 0x01, 0)
                        last_sent_pose = [nx, ny, nz, nrx, nry, nrz]
                    except Exception as e:
                        print("[main] Warning: error sending to piper:", e)

                    # debug output
                    print(f"[data] frame={frame_id} hand={ev.get('hand_id')} rel(um): X={dx} Y={dy} Z={dz}")
                    print(f"       rot(millideg): RX={rx} RY={ry} RZ={rz}  pinch={pinch:.3f}")
                    print(f"       Sent -> X={nx} Y={ny} Z={nz} RX={nrx} RY={nry} RZ={nrz} gripper_units={gripper_units}")
                elif ev_type == "end":
                    print("[leap] tracking END - stopping arm immediately")
                    in_tracking_cycle = False
                    # Immediately put robot in standby (stop)
                    try:
                        piper.ModeCtrl(0x00, 0x01, 0, 0x00)  # ctrl_mode=0x00: Standby
                    except Exception as e:
                        print("[main] Warning: ModeCtrl(standby) failed:", e)
                    # reset state
                    hand_origin = None
                    machine_origin = None
                    last_sent_pose = None
                    # clear stats
                    c_frame_ids_seen.clear()
                    py_frames_processed = 0
                else:
                    # ignore other events
                    pass

            # print fps & stats once per second
            if time.time() - last_stats_time >= 1.0:
                now = time.time()
                secs = now - last_stats_time
                # C fps by counting unique frame ids seen in the last second
                c_fps = len(c_frame_ids_seen) / secs if secs > 0 else 0.0
                py_fps = py_frames_processed / secs if secs > 0 else 0.0
                print(f"[stats] C_fps={c_fps:.2f} events_processed/s={py_fps:.2f} (K_pos={K_POS})")
                # also print current last_sent_pose
                print(f"[stats] last_sent_pose = {last_sent_pose}")
                # reset counters
                last_stats_time = now
                c_frame_ids_seen.clear()
                py_frames_processed = 0

    except KeyboardInterrupt:
        print("User requested stop")
    finally:
        try:
            leap_ext.shutdown()
        except Exception:
            pass
        print("Exiting")

if __name__ == "__main__":
    main()
