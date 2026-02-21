import asyncio
import math
import websockets
import random
import struct
import base64

DRONE_SPEED = 2.0      # m/s along a circle
DRONE_RADIUS = 5.0     # radius of the orbit
DRONE_ALT = 3.0        # Z altitude

_t = 0.0

def get_drone_pose(dt):
    global _t
    _t += dt
    angle = _t * DRONE_SPEED / DRONE_RADIUS
    x = DRONE_RADIUS * math.cos(angle)
    y = DRONE_RADIUS * math.sin(angle)
    z = DRONE_ALT
    yaw = math.degrees(angle + math.pi / 2)  # tangent to circle
    return (x, y, z, yaw)


def get_point_cloud():
    points = []
    for _ in range(1000):
        x = random.uniform(-10, 10)
        y = random.uniform(-10, 10)
        z = random.uniform(0, 5)
        intensity = random.uniform(0, 1)
        points.append((x, y, z, intensity))
    return points


async def handler(websocket):
    print("QGC Connected!")
    try:
        dt = 0.1
        while True:
            drone = get_drone_pose(dt)
            data = get_point_cloud()

            binary_data = bytearray()
            # First 16 bytes: drone position (x, y, z, yaw) as float32 LE
            binary_data.extend(struct.pack('<ffff', *drone))
            for p in data:
                binary_data.extend(struct.pack('<ffff', p[0], p[1], p[2], p[3]))

            b64_payload = base64.b64encode(binary_data).decode('utf-8')

            await websocket.send(b64_payload)
            await asyncio.sleep(dt)

    except websockets.exceptions.ConnectionClosed:
        print("QGC Disconnected")


async def main():
    async with websockets.serve(handler, "0.0.0.0", 8765):
        print("Serving Binary Point Cloud on port 8765 (drone header + 1000 pts)...")
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())
