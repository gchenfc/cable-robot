import serial
import struct
from can_simple import *
from myTimer import MyTimer

def queryPos(ser):
    ser.write(b'0n9c\n')

class MySerial:
    def __init__(self, port='/dev/ttyUSB0'):
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        # self.sentqueue = []
        # self.tosendqueue = []
        # self.resendTimer = MyTimer(0.1, self.resend)
    def __enter__(self):
        self.ser.__enter__()
        return self
    def __exit__(self, type, value, traceback):
        self.ser.__exit__(self, type, value, traceback)

    def update(self):
        pass
        # print(self.sentqueue)
        # if len(self.sentqueue) > 0:
        #     self.resendTimer.update()
        #     return
        # if len(self.tosendqueue) > 0:
        #     tosend = self.tosendqueue.pop()
        #     self.trysend(*tosend)

    def readCan(self, robot_callback):
        msg = self.ser.read_until(expected=bytes.fromhex('FF 00 FF'))
        if len(msg) > 0:
            if len(msg) == 14:
                checksum = sum(msg[:10]) % 0x100
                if msg[10] == checksum:
                    node, cmd, data = msg[0], msg[1], msg[2:10]
                    # for sent in self.sentqueue:
                    #     if sent[0] + 10 == node and sent[1] == cmd:
                    #         # print(sent[2:], data)
                    #         self.sentqueue.remove(sent)
                    #         return
                    if (10 <= node < 14): # ack msg
                        pass
                    if node == 8 and cmd == 31:
                        print('\tMCU parse error')
                        return
                    robot_callback(node, cmd, data)
                else:
                    print('parse error (checksum): {:d}!={:d}, {}'.format(checksum, msg[10], str(msg)))
            else:
                print('parse error (length): ' + str(msg))

    def bytepacking(self, cmd, data):
        if cmd == MSG_CO_NMT_CTRL or cmd == MSG_ODRIVE_HEARTBEAT or cmd == MSG_CO_HEARTBEAT_CMD:
            print('bad cmd')
            return
        elif cmd == MSG_ODRIVE_ESTOP or cmd == MSG_GET_MOTOR_ERROR or cmd == MSG_GET_ENCODER_ERROR or \
             cmd == MSG_GET_SENSORLESS_ERROR or cmd == MSG_GET_ENCODER_ESTIMATES or \
             cmd == MSG_GET_ENCODER_COUNT or cmd == MSG_GET_IQ or cmd == MSG_GET_SENSORLESS_ESTIMATES or \
             cmd == MSG_GET_VBUS_VOLTAGE or cmd == MSG_START_ANTICOGGING or cmd == MSG_RESET_ODRIVE or \
             cmd == MSG_CLEAR_ERRORS:
            assert(data is None)
            return [0 for _ in range(8)]
        elif cmd == MSG_SET_AXIS_NODE_ID or \
             cmd == MSG_SET_AXIS_REQUESTED_STATE:
            return struct.pack('<2i', *[data], 0)
        elif cmd == MSG_SET_AXIS_STARTUP_CONFIG:
            print('not implemented yet')
        elif cmd == MSG_SET_CONTROLLER_MODES:
            return struct.pack('<2i', *data)
        elif cmd == MSG_SET_INPUT_POS:
            return struct.pack('<fhh', *data)
        elif cmd == MSG_SET_INPUT_VEL or \
             cmd == MSG_SET_TRAJ_ACCEL_LIMITS:
            return struct.pack('<2f', *data)
        elif cmd == MSG_SET_INPUT_TORQUE or \
             cmd == MSG_SET_VEL_LIMIT or \
             cmd == MSG_SET_TRAJ_VEL_LIMIT or \
             cmd == MSG_SET_TRAJ_INERTIA:
            return struct.pack('<2f', *data, 0)
        print('bad cmd')

    def writeCan(self, node, cmd, data=None, islist=False, ackdesired=False):
        if not islist:
            data = self.bytepacking(cmd, data)
            self.trysend(node, cmd, data)
            self.trysend(8, 1, [0 for _ in range(8)])
        else:
            if len(node) == 0:
                print('empty list - not sending')
                return
            for node1, cmd1, data1 in zip(node, cmd, data):
                self.trysend(node1, cmd1, self.bytepacking(cmd1, data1))
            self.trysend(8, 1, [0 for _ in range(8)])
            print('sending multiple messages:', node, cmd, data)

    def trysend(self, node, cmd, data, ackdesired=False):
        # if len(self.sentqueue) > 0:
        #     self.tosendqueue.append((node, cmd, *data, ackdesired))
        # else:
        self.writeRaw(bytes([node, cmd, *data]))
        # if ackdesired:
        #     self.sentqueue.append((node, cmd, *data))
    # def resend(self):
    #     for dat in self.sentqueue:
    #         self.writeRaw(bytes(dat))

    def writeRaw(self, msg: bytes):
        checksum = sum(msg) % 0x100
        msg += bytes([checksum])
        msg += bytes.fromhex('FF 00 FF')
        print(checksum, msg[0:10])
        self.ser.write(msg)

if __name__ == "__main__":
    from myTimer import MyTimer
    def callback(node, cmd, data):
        print('node {:d} - cmd {:d} - data {}'.format(node, cmd, data.hex()))
    with MySerial() as ser:
        queryTimer = MyTimer(0.5, lambda: ser.writeCan(0, 9))
        while True:
            queryTimer.update()
            ser.readCan(callback)
