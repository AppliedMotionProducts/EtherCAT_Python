import pysoem
import ctypes
import time
import threading

pd_thread_stop_event = threading.Event()
master = pysoem.Master()
master.open("\\Device\\NPF_{'your adapter id'}") # find your adapter id using Connection_Setup.py program
actual_wkc = 0

#TxPDO
class InputPdo(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('Errorword', ctypes.c_uint16),
        ('statusword', ctypes.c_uint16),
        ('position_actual_value', ctypes.c_int32),
        ('velocity_actual_value', ctypes.c_int32),
        ('followerrorcodevalue', ctypes.c_uint32)
    ]

#RxPDO
class OutputPdo(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('controlword', ctypes.c_uint16),
        ('modes_of_operation', ctypes.c_int8),
        ('target_position', ctypes.c_int32),
        ('target_velocity', ctypes.c_int32),
    ]


modes_of_operation = {
    'No mode': 0,
    'Profile position mode': 1,
    'Profile velocity mode': 3,
    'Profile_torque_mode': 4,
    'Homing mode': 6,
    'Cyclic synchronous position mode': 8,
    'Cyclic synchronous velocity mode': 9,
    'Cyclic synchronous torque mode': 10,
    'Q_mode': -1, # manufacturer specific mode
}


def config_func():
    global device
    # profile velocity
    device.sdo_write(0x6081, 0, bytes(ctypes.c_int32(40000))) #check command pulses per rev, for some products it is 10000 = rev
    # profile acceleration
    device.sdo_write(0x6083, 0, bytes(ctypes.c_int32(50000)))
    # profile deceleration
    device.sdo_write(0x6084, 0, bytes(ctypes.c_int32(50000)))

def processdata_thread():
    global master  # not sure if this is necessary
    global pd_thread_stop_event  # not sure if this is necessary
    global actual_wkc  # not sure if this is necessary
    while not pd_thread_stop_event.is_set():
        master.send_processdata()
        actual_wkc = master.receive_processdata(10000)
        if not actual_wkc == master.expected_wkc:
            print('incorrect wkc')
        time.sleep(0.01)


if master.config_init() > 0:
    device = master.slaves[0]
    device.config_func = config_func()
    master.config_map()

    if master.state_check(pysoem.SAFEOP_STATE, 50_000) == pysoem.SAFEOP_STATE:
        master.state = pysoem.OP_STATE

        proc_thread = threading.Thread(target=processdata_thread)
        proc_thread.start()

        master.send_processdata()  # this is actually done in the "processdata_thread" - maybe it is not needed here
        master.receive_processdata(2000)  # this is actually done in the "processdata_thread" - maybe it is not needed here

        master.write_state()
        master.state_check(pysoem.OP_STATE, 5_000_000)

        if master.state == pysoem.OP_STATE:
            output_data = OutputPdo()
            output_data.modes_of_operation = modes_of_operation['Profile position mode']
            output_data.target_position = 155555  # counts ( 20000 = 1 rev/sec)


            for control_cmd in [6,7,15,31]:  # the 31 is necessary to update the "Target position", search you manual for "New set-point" - only required in pp mode
                output_data.controlword = control_cmd
                device.output = bytes(output_data)  # that is the actual change of the PDO output data
                master.send_processdata()
                master.receive_processdata(1_000)
                time.sleep(0.05)  # you may need to increase this sleep

            try:
                while 1:
                    master.send_processdata()
                    master.receive_processdata(1_000)
                    time.sleep(0.05)
            except KeyboardInterrupt:
                print('stopped')
            # zero everything
            device.output = bytes(len(device.output))
            master.send_processdata()
            master.receive_processdata(1_000)
            pd_thread_stop_event.set()
            proc_thread.join()

        else:
            print('failed to got to OP_STATE')

    else:
        print('failed to got to safeop state')
    master.state = pysoem.PREOP_STATE
    master.write_state()
else:
    print('no device found')

master.close()
