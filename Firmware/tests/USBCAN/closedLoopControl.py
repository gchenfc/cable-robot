import socket
import can
import struct
import contextlib
import time
from myTimer import MyTimer
from functools import partial

# CAN safe shutdown
@contextlib.contextmanager
def make_can(**kwargs):
    try:
        bus = can.Bus(bustype='socketcan', channel='can0', bitrate=500000, **kwargs)
        yield bus
    finally:
        print('  exiting')
        bus.stop_all_periodic_tasks(remove_tasks=True)
        bus.shutdown()

@contextlib.contextmanager
def make_can():
    try:
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind(("can0",))
        sock.settimeout(0)
        yield sock
    finally:
        pass

sock_fmt = "<IB3x8s"

def send_can(sock, node, cmd, data, rtr=False):
    can_id = nodecmd_to_id(node, cmd)
    if rtr:
        can_id |= socket.CAN_RTR_FLAG
    can_pkt = struct.pack(sock_fmt, can_id, 8, data)
    sock.send(can_pkt)
def read_can(sock):
    try:
        can_pkt = sock.recv(16)
    except:
        return None, None, None
    can_id, length, data = struct.unpack(sock_fmt, can_pkt)
    can_id &= socket.CAN_EFF_MASK
    data = data[:length]
    return parseMsg(can_id, data)

def nodecmd_to_id(node, cmd):
    return (node << 5) | cmd
def parseMsg(can_id, data):
    cmd = can_id % 0x20
    if cmd == 0x09 or cmd == 0x14:
        data = struct.unpack('<2f', data)
    elif cmd == 0x01:
        data = struct.unpack('<2i', data)
    else:
        data = struct.unpack('<2i', data)
    return can_id >> 5,\
           cmd, \
           data

class MyListener(can.Listener):
    def on_error(self, exc):
        print('exc! ', exc)
    def on_message_received(self, msg):
        # print(msg)
        # return
        if not msg.is_remote_frame:
            node, cmd, data = parseMsg(msg)
            print('node: {:2d}, cmd: 0x{:02x}, data: {:6.3f} ** {:6.3f}'.format(node, cmd, *data))
    def stop(self):
        print('stopping')

def query(sock, node, cmd):
    send_can(sock, node, cmd, b'', rtr=True)
def motor_on(bus, axis):
    msg = can.Message(arbitration_id=nodecmd_to_id(axis, 0x09),
                      is_remote_frame=False,
                      is_extended_id=False)
    bus.send(msg, timeout=1)
    print('sent')
def motor_off(bus, axis):
    bus.send(can.Message(arbitration_id=nodecmd_to_id(axis, 0x07),
                         is_extended_id=False,
                         data=[0,0,0,0,1,0,0,0]))
def controller_mode(bus, axis, mode):
    bus.send(can.Message(arbitration_id=nodecmd_to_id(axis, 0x0B),
                         is_extended_id=False,
                         data=[0,0,0,mode,0,0,0,1]))

count = 0
def fps_count():
    global count
    print(count)
    count = 0

def main():
    filters = None
    filters = [{"can_id": cmd, "can_mask": 0x7F, "extended": False} for cmd in
                [0x01, 0x07]]
    print('hello there!!!')

    with make_can() as sock:
        global count
        count = 0
        prev_time = time.time()
        queryTimers = [MyTimer(1/250, partial(query, sock, node, 0x09)) for node in range(4)]
        fpsTimer = MyTimer(1, fps_count)
        while True:
            node, cmd, data = read_can(sock)
            if node is not None:
                if (cmd == 0x09):
                    count += 1
                # print('node: {:2d}, cmd: 0x{:02x}, data: {:6.3f} ** {:6.3f}'.format(node, cmd, *data))
            for queryTimer in queryTimers:
                queryTimer.update()
            fpsTimer.update()
    return
    with make_can(can_filters=filters) as bus:
        notifier = can.Notifier(bus, [MyListener()])
        
        time.sleep(1)

        # msg = can.Message(arbitration_id=nodecmd_to_id(0, 0x07),
        #                   is_remote_frame=False,
        #                   is_extended_id=False,
        #                   data=[8,0,0,0,0,0,0,0]) # encoder estimates
        # bus.send(msg)
        # print('sent!!!')
        # motor_on(bus, 0)
        # controller_mode(bus, 0, 1) # 1: torque

        for node in range(4):
            msg = can.Message(arbitration_id=nodecmd_to_id(node, 0x09), is_remote_frame=True, is_extended_id=False) # encoder estimates
            querier = bus.send_periodic(msg, 0.01, duration=None, store_task=True)
            # msg = can.Message(arbitration_id=nodecmd_to_id(node, 0x14), is_remote_frame=True, is_extended_id=False) # current measurements
            # querier = bus.send_periodic(msg, 0.01, duration=None, store_task=True)
            pass

        # motor_on(bus, 0)
        time.sleep(2)

        # motor_off(bus, 0)

        time.sleep(2)

        querier.stop()
        notifier.stop()

if __name__ == '__main__':
  main()