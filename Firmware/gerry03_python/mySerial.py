import serial

def queryPos(ser):
    ser.write(b'0n9c\n')

class MySerial:
    def __init__(self, port='/dev/ttyUSB0'):
        self.ser = serial.Serial(port, 115200, timeout=0.1)
    def __enter__(self):
        self.ser.__enter__()
        return self
    def __exit__(self, type, value, traceback):
        self.ser.__exit__(self, type, value, traceback)

    def readCan(self, robot_callback):
        msg = self.ser.read_until(expected=bytes.fromhex('FF 00 FF'))
        if len(msg) > 0:
            if len(msg) == 13:
                node, cmd, data = msg[0], msg[1], msg[2:10]
                robot_callback(node, cmd, data)
            else:
                try:
                    if msg[:17].decode() == 'Affirmed multiple':
                        robot_callback(None, None, None, ack=True)
                    else:
                        print('parse error: ' + str(msg))
                except:
                    print('parse error: ' + str(msg))

    def writeCan(self, node, cmd, data=None, islist=False):
        if not islist:
            out = '{:d}n{:d}c'.format(node, cmd)
            if data:
                if not (isinstance(data, list) or isinstance(data, tuple)):
                    data = [data]
                for datum in data:
                    out += '{},'.format(datum)
                out = out[:-1]
            self.writeRaw(out)
        else:
            if len(node) == 0:
                print('empty list - not sending')
                return
            outAll = ''
            for node, cmd, data in zip(node, cmd, data):
                out = '{:d}n{:d}c'.format(node, cmd)
                if data:
                    if not (isinstance(data, list) or isinstance(data, tuple)):
                        data = [data]
                    for datum in data:
                        out += '{},'.format(round(datum, 3))
                    out = out[:-1]
                outAll += out + '~'
            print('sending multiple messages:', outAll[:-1])
            self.writeRaw(outAll[:-1])
    
    def writeRaw(self, msg: str, debug=False):
        checksum = sum(msg.encode('raw_unicode_escape')) % 0x100
        msg += chr(checksum)
        msg += '\n'
        if debug:
            print(msg.encode('raw_unicode_escape'))
        else:
            self.ser.write(msg.encode('raw_unicode_escape'))

if __name__ == "__main__":
    from myTimer import MyTimer
    def callback(node, cmd, data):
        print('node {:d} - cmd {:d} - data {}'.format(node, cmd, data.hex()))
    with MySerial() as ser:
        queryTimer = MyTimer(0.5, lambda: ser.writeCan(0, 9))
        while True:
            queryTimer.update()
            ser.readCan(callback)
