import asyncio
import socket
import websockets

# Not complete or tested - just copy-pasted from chatGPT

def main():
    UDP_IP = socket.gethostbyname(socket.gethostname())
    UDP_PORT = 7709

    # Create a list to store connected websockets
    connected_websockets = set()

    async def forward_udp_to_websocket(websocket, path):
        connected_websockets.add(websocket)
        try:
            async for message in websocket:
                pass
        finally:
            connected_websockets.remove(websocket)

    # create a UDP socket
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind(('0.0.0.0', 1234))
    udp_sock.bind((UDP_IP, UDP_PORT))

    async def receive_udp():
        while True:
            data, addr = udp_sock.recvfrom(1024)
            for websocket in connected_websockets:
                await websocket.send(data.decode())

    start_server = websockets.serve(forward_udp_to_websocket, 'localhost', 8765)

    asyncio.get_event_loop().create_task(start_server)
    asyncio.get_event_loop().create_task(receive_udp())
    asyncio.get_event_loop().run_forever()


if __name__ == '__main__':
    main()
