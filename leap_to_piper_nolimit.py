#!/usr/bin/env python3
# leap_to_piper_nolimit.py
# No smoothing / no step limits — send target immediately to Piper.
# Position: incremental control; Orientation: absolute; Gripper: absolute (mapped & inverted)
# Prints C-frame-rate and Python processing fps once per second.

import time
import math
import sys

# Config
CAN_PORT = "can0"
LOOP_DT = 0.01               # control loop (s) - sleep between cycles
K_POS = 1.0                  # scale for position mapping (X_machine = X_machine0 + K_POS * delta_hand)
GRIPPER_FULL_RANGE_MM = 100.0
MOVE_SPEED_PERCENT = 80

# Derived units
M_TO_PIPER_POS = 1e6         # meters -> Piper pos unit (0.001 mm)
DEG_TO_MILLIDEG = 1000       # degrees -> millideg (0.001 deg)

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
    # pinch_strength in [0,1]. Map to 0..GRIPPER_FULL_RANGE_MM (mm). Invert because open/close reversed.
    mm = pinch_strength * GRIPPER_FULL_RANGE_MM
    inv_mm = GRIPPER_FULL_RANGE_MM - mm
    units = int(round(inv_mm * 1000.0))  # piper expects 0.001mm units
    return max(0, units)

def wrap_angle_md(x):
    """把角度限制到主值域 [-180000, 180000] millideg"""
    return int(((int(x) + 180000) % 360000) - 180000)

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
        piper.ModeCtrl(0x01, 0x00, 30, 0x00)  # CAN control, move mode P (or keep as you prefer)
        # Use commonly safe neutral command — you may change to JointCtrl zero if desired
        # Sending a plausible EndPose Ctrl as an example neutral pose (adjust as needed)
        # (These numbers are example placeholders; if you have a preferred zero pose, set them)
        try:
            piper.EndPoseCtrl(257384, 4793, 147021, -173360, 65070, -169444)
        except Exception:
            # some SDK/firmware combos might prefer JointCtrl; ignore if fails
            pass
        piper.GripperCtrl(0, 1000, 0x01, 0)   # enable gripper position 0
    except Exception as e:
        print("[main] Warning: zeroing commands raised:", e)

    # init leap
    try:
        leap_ext.init()
    except Exception as e:
        print("[main] leap_ext.init() failed (continuing if already initialized):", e)
    print("[main] Initialization done. Entering loop.")

    # state
    in_tracking_cycle = False
    machine_origin = None        # machine origin: tuple of piper units (X,Y,Z,RX,RY,RZ)
    last_sent_pose = None        # last sent [X,Y,Z,RX,RY,RZ] in piper units

    # fps stats
    py_frames_processed = 0
    c_frame_ids_seen = set()
    last_stats_time = time.time()
    time.sleep(0.5)

    try:
        while True:
            # take newest event (we use small blocking timeout)
            ev = leap_ext.next_event(10)  # ms
            tnow = time.time()

            if ev is None:
                # no event; still allow a small sleep to not busy-loop
                time.sleep(LOOP_DT)
            else:
                ev_type = ev.get("event") if isinstance(ev, dict) else None

                if ev_type == "connected":
                    print("[leap] connected")
                elif ev_type == "disconnected":
                    print("[leap] disconnected - clearing state")
                    in_tracking_cycle = False
                    machine_origin = None
                    last_sent_pose = None
                elif ev_type == "start":
                    print(f"[leap] START frame={ev.get('frame_id')} hand={ev.get('hand_id')}")
                    in_tracking_cycle = True
                    mo = read_arm_endpose_safe(piper, timeout_s=1.0)
                    if mo is None:
                        print("[main] Warning: cannot read machine endpose; assuming zeros")
                        mo = (0,0,0,0,0,0)
                    machine_origin = mo
                    last_sent_pose = list(machine_origin)
                    py_frames_processed = 0
                    c_frame_ids_seen.clear()
                elif ev_type == "data" and in_tracking_cycle:
                    frame_id = ev.get("frame_id")
                    c_frame_ids_seen.add(frame_id)
                    py_frames_processed += 1

                    # relative deltas in μm (same unit as Piper pos unit)
                    dx = int(ev.get("X", 0))
                    dy = int(ev.get("Y", 0))
                    dz = int(ev.get("Z", 0))

                    # RX/RY/RZ are provided in millideg (absolute, per C extension remap)
                    RX_mdeg = int(ev.get("RX", 0))-180000
                    RY_mdeg = (-int(ev.get("RY", 0)))+90000
                    RZ_mdeg = int(ev.get("RZ", 0))-180000

                    # If your C mapping needs additional manual adjustments (flips/offsets),
                    # do them here. This example assumes raw values are usable after optional tweaks.
                    # For historic tweaks you used, you can apply them — here we keep raw and wrap:
                    rx = wrap_angle_md(RX_mdeg)
                    ry = wrap_angle_md(RY_mdeg)
                    rz = wrap_angle_md(RZ_mdeg)

                    pinch = float(ev.get("pinch_strength", 0.0))

                    # machine_origin must exist
                    if machine_origin is None:
                        mo = read_arm_endpose_safe(piper, timeout_s=0.5)
                        if mo is None:
                            machine_origin = (0,0,0,0,0,0)
                        else:
                            machine_origin = mo
                            last_sent_pose = list(machine_origin)

                    # compute targets directly (no smoothing)
                    tgt_X = int(round(machine_origin[0] + K_POS * dx))
                    tgt_Y = int(round(machine_origin[1] + K_POS * dy))
                    tgt_Z = int(round(machine_origin[2] + K_POS * dz))

                    tgt_RX = int(rx)
                    tgt_RY = int(ry)
                    tgt_RZ = int(rz)

                    # gripper
                    gripper_units = map_gripper_units_from_pinch(pinch)

                    # send immediately (no smoothing)
                    try:
                        piper.ModeCtrl(0x01, 0x02, MOVE_SPEED_PERCENT, 0x00)  # CAN control, move L (linear)
                        piper.EndPoseCtrl(tgt_X, tgt_Y, tgt_Z, tgt_RX, tgt_RY, tgt_RZ)
                        piper.GripperCtrl(abs(gripper_units), 1000, 0x01, 0)
                        last_sent_pose = [tgt_X, tgt_Y, tgt_Z, tgt_RX, tgt_RY, tgt_RZ]
                    except Exception as e:
                        print("[main] Warning: error sending to piper:", e)

                    # debug output
                    print(f"[data] frame={frame_id} hand={ev.get('hand_id')} rel(um): X={dx} Y={dy} Z={dz}")
                    print(f"       rot(millideg): RX={RX_mdeg} RY={RY_mdeg} RZ={RZ_mdeg}  pinch={pinch:.3f}")
                    print(f"       Sent -> X={tgt_X} Y={tgt_Y} Z={tgt_Z} RX={tgt_RX} RY={tgt_RY} RZ={tgt_RZ} gripper_units={gripper_units}")

                elif ev_type == "end":
                    print("[leap] tracking END - stopping arm immediately")
                    in_tracking_cycle = False
                    # Immediately put robot in standby (stop)
                    try:
                        piper.ModeCtrl(0x00, 0x01, 0, 0x00)  # ctrl_mode=0x00: Standby
                    except Exception as e:
                        print("[main] Warning: ModeCtrl(standby) failed:", e)
                    machine_origin = None
                    last_sent_pose = None
                    c_frame_ids_seen.clear()
                    py_frames_processed = 0
                else:
                    # ignore other events
                    pass

            # print fps & stats once per second
            if time.time() - last_stats_time >= 1.0:
                now = time.time()
                secs = now - last_stats_time
                c_fps = len(c_frame_ids_seen) / secs if secs > 0 else 0.0
                py_fps = py_frames_processed / secs if secs > 0 else 0.0
                print(f"[stats] C_fps={c_fps:.2f} events_processed/s={py_fps:.2f} (K_pos={K_POS})")
                print(f"[stats] last_sent_pose = {last_sent_pose}")
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
