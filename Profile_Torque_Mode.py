"""
Applied Motion Products EtherCAT - Profile Torque (TQ, mode 4) test via pysoem.

Profile Torque is a PROFILE mode: you give the drive a target torque and a
torque slope, and the drive ramps to it internally on its own clock. It needs
NO Distributed Clock / Sync0, so it works from pysoem on Windows (unlike CST).

Applies a fixed torque percentage and holds it for a set time, then disables.
Adjust the NIC string in master.open() to your adapter.
"""

import pysoem
import ctypes
import struct
import time
import threading

pd_thread_stop_event = threading.Event()
master = pysoem.Master()
master.open("\\Device\\NPF_{0ADF189E-D615-4401-95C4-8AB3AE36801C}")

# ---------------- tunables ----------------
TORQUE_PERCENT = 5.0       # % of rated torque
HOLD_SECONDS   = 15         # how long to hold torque (60 = 1 min)
TORQUE_SLOPE   = 500        # 0x6087 torque ramp rate, 0.1%/s (500 = 50%/s; drive ramps internally)
SPEED_LIMIT    = 10000    # 0x2A47 torque-mode speed limit, counts/s
TORQUE_LIMIT   = 1000       # 0x60E0/0x60E1 pos/neg torque limit, 0.1% (so torque isn't clamped)
BRAKE_DELAY    = 0.5        # s to wait after enable for the holding brake to open
TQ_MODE        = 4          # Profile Torque mode

TARGET = int(round(TORQUE_PERCENT * 10))   # 0x6071 is in 0.1% -> 20% = 200


# ---------------- PDO structures ----------------
class InputPdo(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('errorword',                  ctypes.c_uint16),  # 0x603F
        ('statusword',                 ctypes.c_uint16),  # 0x6041
        ('modes_of_operation_display', ctypes.c_int8),    # 0x6061
        ('torque_actual_value',        ctypes.c_int16),   # 0x6077
        ('velocity_actual_value',      ctypes.c_int32),   # 0x606C
        ('position_actual_value',      ctypes.c_int32),   # 0x6064
    ]


class OutputPdo(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('controlword',        ctypes.c_uint16),  # 0x6040
        ('modes_of_operation', ctypes.c_int8),    # 0x6060
        ('target_torque',      ctypes.c_int16),   # 0x6071
    ]


# ---------------- helpers ----------------
def write_pdo_map(pdo_idx, entries):
    device.sdo_write(pdo_idx, 0, struct.pack('B', 0))
    for i, (oi, os, bl) in enumerate(entries, start=1):
        device.sdo_write(pdo_idx, i, struct.pack('<I', (oi << 16) | (os << 8) | bl))
    device.sdo_write(pdo_idx, 0, struct.pack('B', len(entries)))


def assign_pdo(ai, pdos):
    device.sdo_write(ai, 0, struct.pack('B', 0))
    for i, p in enumerate(pdos, start=1):
        device.sdo_write(ai, i, struct.pack('<H', p))
    device.sdo_write(ai, 0, struct.pack('B', len(pdos)))


def config_func():
    # PRE-OP: remap PDOs to carry torque, select TQ mode, set ramp + limits.
    assign_pdo(0x1C12, [])
    write_pdo_map(0x1600, [(0x6040, 0, 16), (0x6060, 0, 8), (0x6071, 0, 16)])
    assign_pdo(0x1C12, [0x1600])

    assign_pdo(0x1C13, [])
    write_pdo_map(0x1A00, [(0x603F, 0, 16), (0x6041, 0, 16), (0x6061, 0, 8),
                           (0x6077, 0, 16), (0x606C, 0, 32), (0x6064, 0, 32)])
    assign_pdo(0x1C13, [0x1A00])

    device.sdo_write(0x6060, 0, struct.pack('b', TQ_MODE))        # Profile Torque
    device.sdo_write(0x6087, 0, struct.pack('<I', TORQUE_SLOPE))  # torque slope (internal ramp)
    device.sdo_write(0x2A47, 0, struct.pack('<I', SPEED_LIMIT))   # torque-mode speed limit
    device.sdo_write(0x60E0, 0, struct.pack('<H', TORQUE_LIMIT))  # positive torque limit
    device.sdo_write(0x60E1, 0, struct.pack('<H', TORQUE_LIMIT))  # negative torque limit


def processdata_thread():
    while not pd_thread_stop_event.is_set():
        master.send_processdata()
        master.receive_processdata(100_000)
        time.sleep(0.001)


def rds(idx, sub=0):
    return int.from_bytes(device.sdo_read(idx, sub), 'little', signed=True)


# ---------------- main ----------------
if master.config_init() > 0:
    device = master.slaves[0]
    device._disable_complete_access()     # this drive rejects CompleteAccess reads
    config_func()
    master.config_map()
    # NO Distributed Clock needed - TQ is a profile mode

    if master.state_check(pysoem.SAFEOP_STATE, 50_000) == pysoem.SAFEOP_STATE:
        master.state = pysoem.OP_STATE
        proc_thread = threading.Thread(target=processdata_thread)
        proc_thread.start()
        master.write_state()
        master.state_check(pysoem.OP_STATE, 5_000_000)

        if master.state == pysoem.OP_STATE:
            out = OutputPdo()
            out.modes_of_operation = TQ_MODE
            out.target_torque = 0

            # enable: 0x06 -> 0x07 -> 0x0F (also releases the brake)
            for cmd in (6, 7, 15):
                out.controlword = cmd
                device.output = bytes(out)
                time.sleep(0.1)
                inp = InputPdo.from_buffer_copy(device.input)
                print("cmd=0x%02X status=0x%04X mode=%d" %
                      (cmd, inp.statusword, inp.modes_of_operation_display))

            inp = InputPdo.from_buffer_copy(device.input)
            if (inp.statusword & 0x6F) != 0x27:
                print("[!] NOT Operation Enabled (0x%04X) - check STO / hardware enable" % inp.statusword)

            # hold 0 torque while the holding brake physically opens
            print("[+] enabled - holding 0 torque %.1fs for brake to release..." % BRAKE_DELAY)
            t0 = time.time()
            while time.time() - t0 < BRAKE_DELAY:
                out.controlword = 15
                out.target_torque = 0
                device.output = bytes(out)
                time.sleep(0.02)

            # apply target torque; the drive ramps to it internally (0x6087)
            print("[+] Profile Torque %.1f%% (0x6071=%d), slope=%d (0.1%%/s), for %d s. Ctrl+C to stop." %
                  (TORQUE_PERCENT, TARGET, TORQUE_SLOPE, HOLD_SECONDS))
            start = time.time()
            try:
                while time.time() - start < HOLD_SECONDS:
                    out.controlword = 15
                    out.modes_of_operation = TQ_MODE
                    out.target_torque = TARGET
                    device.output = bytes(out)

                    inp = InputPdo.from_buffer_copy(device.input)
                    print("t=%4ds act_trq=%5d vel=%9d cmd6074=%5d status=0x%04X err=0x%04X" %
                          (int(time.time() - start), inp.torque_actual_value, inp.velocity_actual_value,
                           rds(0x6074), inp.statusword, inp.errorword))

                    if (inp.statusword & 0x4F) == 0x08:
                        print("[!] drive faulted err=0x%04X - stopping" % inp.errorword)
                        break
                    time.sleep(1.0)
            except KeyboardInterrupt:
                print("\n[+] stopped by user")

            # stop: zero torque, then disable (drive re-engages the brake)
            out.target_torque = 0
            device.output = bytes(out)
            time.sleep(0.2)
            out.controlword = 0
            device.output = bytes(out)
            time.sleep(0.2)
            device.output = bytes(len(device.output))
        else:
            print('failed to reach OP_STATE')
    else:
        print('failed to reach SAFEOP_STATE')

    pd_thread_stop_event.set()
    proc_thread.join()
    master.state = pysoem.PREOP_STATE
    master.write_state()
else:
    print('no device found')

master.close()