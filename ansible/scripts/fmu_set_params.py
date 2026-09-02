#!/usr/bin/env python3
"""Set (or audit) PX4 parameters on the FMU over the USB-C MAVLink link, taking the values
from the fleet's QGroundControl parameter file so that file stays the single source of truth.

    fmu_set_params.py --from-file nxt_params.params --names EKF2_TAU_POS,EKF2_DELAY_MAX [--check]

Transport: /dev/ttyACM0 (falls back to /dev/ttyACM1 -- PX4 re-enumerates after an FC reboot),
115200 baud, MAVLink auto-enabled on USB by SYS_USB_AUTO=2. The DDS link on TELEM2 is untouched.
Refuses to write while the vehicle is armed. Every write is read back and compared; PX4 persists
parameters itself on PARAM_SET, so no save/reboot step is issued. EKF2 re-reads these at runtime.
Exit 0 = every requested parameter reads back at the requested value; 2 = mismatch; 3 = link.
"""
import argparse, glob, json, math, sys, time

QGC_TYPE = {6: 'INT32', 9: 'REAL32'}          # MAV_PARAM_TYPE codes used by the QGC file


def parse_qgc(path, names):
    want = set(names); out = {}
    for line in open(path):
        if line.startswith('#') or not line.strip():
            continue
        f = line.split()
        if len(f) < 5 or f[2] not in want:
            continue
        out[f[2]] = (float(f[3]), QGC_TYPE.get(int(f[4]), 'REAL32'))
    missing = want - set(out)
    if missing:
        sys.exit('parameter(s) not in %s: %s' % (path, ','.join(sorted(missing))))
    return out


def connect(device, baud, timeout):
    from pymavlink import mavutil
    cands = [device] if device else sorted(glob.glob('/dev/ttyACM*'))
    if not cands:
        sys.exit(3)
    for dev in cands:
        try:
            m = mavutil.mavlink_connection(dev, baud=baud)
            if m.wait_heartbeat(timeout=timeout):
                return m, dev
        except Exception:
            pass
    print('no MAVLink heartbeat on %s' % cands, file=sys.stderr)
    sys.exit(3)


def read_param(m, name, timeout):
    from pymavlink import mavutil
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=timeout)
        if msg and msg.param_id.rstrip('\0') == name:
            return msg
    return None


def same(a, b, ptype):
    return int(round(a)) == int(round(b)) if ptype == 'INT32' else math.isclose(a, b, rel_tol=1e-5, abs_tol=1e-7)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--from-file', required=True)
    ap.add_argument('--names', required=True, help='comma-separated PX4 parameter names')
    ap.add_argument('--device', default='', help='serial device (default: auto /dev/ttyACM*)')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--timeout', type=float, default=10.0)
    ap.add_argument('--check', action='store_true', help='read and compare only, never write')
    ap.add_argument('--parse-only', action='store_true', help='print the parsed targets and exit (no FMU)')
    a = ap.parse_args()
    targets = parse_qgc(a.from_file, [n for n in a.names.split(',') if n])
    if a.parse_only:
        print(json.dumps({k: {'value': v[0], 'type': v[1]} for k, v in targets.items()}, indent=1)); return 0
    from pymavlink import mavutil
    m, dev = connect(a.device, a.baud, a.timeout)
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=a.timeout)
    armed = bool(hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
    if armed and not a.check:
        print(json.dumps({'device': dev, 'error': 'vehicle is ARMED; refusing to write parameters'})); return 2
    rows = []; bad = 0
    for name, (val, ptype) in targets.items():
        before = read_param(m, name, a.timeout)
        bval = before.param_value if before else None
        after = bval
        if not a.check and (bval is None or not same(bval, val, ptype)):
            ptype_id = mavutil.mavlink.MAV_PARAM_TYPE_INT32 if ptype == 'INT32' else mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(val), ptype_id)
            ack = read_param(m, name, a.timeout)   # PX4 replies with PARAM_VALUE after a set
            after = ack.param_value if ack else None
        ok = after is not None and same(after, val, ptype)
        bad += 0 if ok else 1
        rows.append({'name': name, 'target': val, 'before': bval, 'after': after, 'ok': ok})
    print(json.dumps({'device': dev, 'sysid': m.target_system, 'armed': armed, 'mode': 'check' if a.check else 'set',
                      'params': rows, 'all_ok': bad == 0}))
    return 0 if bad == 0 else 2


if __name__ == '__main__':
    sys.exit(main())
