import serial
import time
import struct
from myTimer import MyTimer
from can_simple import *

def parseInput(msg: bytes):
    node, cmd, data = msg[0], msg[1], msg[2:10]
    # print(node, cmd, data)
    f1, f2 = struct.unpack('<2f', data)  # little endian, 2 floats
    i1, i2 = struct.unpack('<2i', data)  # little endian, 2 ints (4 bytes)
    p, v, t = struct.unpack('<fhh', data)  # little endian, float-short-short
    toPrint = 'node {:d} - {:30s}'.format(node, CMDSTR[cmd])
    if (cmd == MSG_ODRIVE_HEARTBEAT):
        toPrint +='\theartbeat'
        return
    elif (cmd in [MSG_ODRIVE_ESTOP,
                MSG_START_ANTICOGGING,
                MSG_RESET_ODRIVE,
                MSG_CLEAR_ERRORS]):
        toPrint +='\tThis message should not be coming from the slave'
    # query info
    elif (cmd == MSG_GET_MOTOR_ERROR):
        toPrint +='\tmotor error: {}'.format(i1)
    elif (cmd == MSG_GET_ENCODER_ERROR):
        toPrint +='\tencoder error: {}'.format(i1)
    elif (cmd == MSG_GET_SENSORLESS_ERROR):
        toPrint +='\tsensorless error: {}'.format(i1)
    elif (cmd == MSG_GET_ENCODER_ESTIMATES):
        toPrint +='\tencoder pos: {:.2f}\tvel: {:.3f}'.format(f1, f2)
    elif (cmd == MSG_GET_ENCODER_COUNT):
        toPrint +='\tencoder shadow: {:.2f}\tcount: {:.3f}'.format(i1, i2)
    elif (cmd == MSG_GET_IQ):
        toPrint +='\tcurrent setpoint: {:.2f}\tmeasured: {:.2f}'.format(f1, f2)
    elif (cmd == MSG_GET_SENSORLESS_ESTIMATES):
        toPrint +='\tsensorless pos: {:.2f}\tvel: {:.3f}'.format(f1, f2)
    elif (cmd == MSG_GET_VBUS_VOLTAGE):
        toPrint +='\tvoltage: {:.2f}'.format(f1)
    elif(cmd in [MSG_SET_AXIS_NODE_ID,
                 MSG_SET_AXIS_REQUESTED_STATE,
                 MSG_SET_AXIS_STARTUP_CONFIG,
                 MSG_SET_CONTROLLER_MODES,
                 MSG_SET_INPUT_POS,
                 MSG_SET_INPUT_VEL,
                 MSG_SET_INPUT_TORQUE,
                 MSG_SET_VEL_LIMIT,
                 MSG_SET_TRAJ_VEL_LIMIT,
                 MSG_SET_TRAJ_ACCEL_LIMITS,
                 MSG_SET_TRAJ_INERTIA]):
        toPrint +='\tThis message should not be coming from the slave'
    else:
        toPrint +='\tCommand not recognized!'
    print(toPrint)

def queryPos(ser):
    ser.write(b'0n9c\n')

def main():
    with serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1) as ser:
        print('opened {}'.format(ser.name))
        queryTimer = MyTimer(0.5, lambda: queryPos(ser))
        while True:
            queryTimer.update()
            x = ser.read_until(expected=bytes.fromhex('FF 00 FF'))
            if len(x) > 0:
                if len(x) == 13:
                    parseInput(x)
                else:
                    print('parse error: ' + str(x) + x.hex())
    print('exiting')

if __name__ == "__main__":
    main()
