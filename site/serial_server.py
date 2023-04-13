'''Serial server for WebSerial API
This code is a standalone program for forwarding Serial ports to websockets.
Run this on a computer connected over USB to a serial device, then access it by port forwarding 8765
On your local computer, run
    `ssh -L 8765:localhost:8765 user@usb_computer_ip_address`

Then, on your local computer, you can create a websocket and access the serial port:
```js
const ws = new WebSocket('ws://localhost:8765')
ws.onopen = () => {
    ws.send(JSON.stringify({command: 'getPorts'}))
    ws.send(JSON.stringify({command: 'open', port: '/dev/tty.usbmodem14101', baudRate: 9600}))
    ws.send(JSON.stringify({command: 'write', data: 'Hello, world!'}))
    ws.send(JSON.stringify({command: 'read'}))  // poll read
    ws.send(JSON.stringify({command: 'streaming_read'}))
    ws.send(JSON.stringify({command: 'close'}))
}
ws.onmessage = (event) => {
    console.log(JSON.parse(event.data))
}
```

Author: Gerry Chen
Date: Apr 12, 2023
'''

import asyncio
import json
import serial
import serial.tools.list_ports
import websockets


import socket
def get_local_ip():
    # return socket.gethostbyname(socket.gethostname())
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]
print(get_local_ip())

async def stream_serial_data(websocket, ser):
    while ser.is_open:
        data = ser.read(ser.in_waiting)
        if data:
            await websocket.send(json.dumps({'command': 'data stream', 'data': data.decode()}))
        await asyncio.sleep(0.01)


async def handle_serial(websocket, path):
    connections = {}

    async for message in websocket:
        try:
            data = json.loads(message)
            command = data['command']
            ser = connections.get(websocket)

            if command == 'getPorts':
                try:
                    fields = ['device', 'name', 'description', 'hwid', 'vid', 'pid', 'serial_number',
                                'location', 'manufacturer', 'product', 'interface']
                    port2dict = lambda port: {field: getattr(port, field) for field in fields}
                    ports = [port2dict(port) for port in serial.tools.list_ports.comports()]
                    # ports = [port.device for port in serial.tools.list_ports.comports()]
                    print(f'Available ports: {ports}')
                    await websocket.send(json.dumps({'command': 'getPorts', 'ports': ports}))
                except Exception as e:
                    print(f'Error: {e}')
                    await websocket.send(json.dumps({'command': 'getPorts', 'error': str(e)}))

            elif command == 'open':
                try:
                    if ser is None:
                        ser = serial.Serial(data['port'], data['baudRate'], timeout=data['timeout'])
                        print(f'Opened serial port {ser.port}!')
                        connections[websocket] = ser
                        await websocket.send(json.dumps({'command': 'open', 'status': 'connected'}))
                    else:
                        print(f'Port {ser.port} is already open!')
                    await websocket.send(
                        json.dumps({
                            'command': 'open',
                            'error': f'Port {ser.port} is already open!'
                        }))
                except Exception as e:
                    print(f'Error: {e}')
                    await websocket.send(json.dumps({'command': 'open', 'error': str(e)}))

            elif command == 'close':
                try:
                    if ser is not None:
                        ser.close()
                        print(f'Closed serial port {ser.port}!')
                        del connections[websocket]
                        await websocket.send(
                            json.dumps({
                                'command': 'close',
                                'status': 'disconnected'
                            }))
                    else:
                        print("Didn't close port because none are open!")
                        await websocket.send(
                            json.dumps({
                                'command': 'close',
                                'error': "Didn't close port because none are open!"
                            }))
                except Exception as e:
                    print(f'Error: {e}')
                    await websocket.send(json.dumps({'command': 'close', 'error': str(e)}))

            elif command == 'read':
                try:
                    if ser is not None:
                        data = ser.read(ser.in_waiting)
                        await websocket.send(json.dumps({'command': 'read', 'data': data.decode()}))
                    else:
                        print("Didn't read from port because none are open!")
                        await websocket.send(
                            json.dumps({
                                'command': 'read',
                                'error': "Didn't read from port because none are open!"
                            }))
                except Exception as e:
                    print(f'Error: {e}')
                    await websocket.send(json.dumps({'command': 'read', 'error': str(e)}))

            elif command == 'streaming_read':
                try:
                    if ser is not None:
                        asyncio.create_task(stream_serial_data(websocket, ser))
                        await websocket.send(json.dumps({'command': 'streaming_read', 'status': 'streaming'}))
                    else:
                        print("Can't stream from port because none are open!")
                        await websocket.send(
                            json.dumps({
                                'command': 'streaming_read',
                                'error': "Can't stream from port because none are open!"
                            }))
                except Exception as e:
                    print(f'Error: {e}')
                    await websocket.send(json.dumps({'command': 'streaming_read', 'error': str(e)}))

            elif command == 'write':
                try:
                    if ser is not None:
                        print(f'write to {ser.port}: {data["data"]}')
                        ser.write(data['data'].encode())
                        await websocket.send(json.dumps({'command': 'write', 'status': 'written'}))
                    else:
                        print("Didn't write to port because none are open!")
                        await websocket.send(
                            json.dumps({
                                'command': 'write',
                                'error': "Didn't write to port because none are open!"
                            }))
                except Exception as e:
                    print(f'Error: {e}')
                    await websocket.send(json.dumps({'command': 'write', 'error': str(e)}))

        except Exception as e:
            print(f'Error: {e}')
            await websocket.send(json.dumps({'command': 'ERROR', 'error': str(e)}))

    # Close the serial port connection when the WebSocket is closed
    if websocket in connections:
        connections[websocket].close()
        del connections[websocket]


async def main():
    server = await websockets.serve(handle_serial, 'localhost', 8765)
    await server.wait_closed()


if __name__ == '__main__':
    asyncio.run(main())
