import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import numpy as np
import json
import asyncio
import websockets

class PointCloudWebSocket(Node):
    def __init__(self):
        super().__init__('pointcloud_websocket')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/apc/points/data_raw',  # Reemplaza con tu tópico
            self.pointcloud_callback,
            10)
        self.subscription  # Evita que Python lo elimine
        self.websocket_clients = set()
        self.loop = asyncio.get_event_loop()
        self.loop.create_task(self.start_websocket_server())

    def pointcloud_callback(self, msg):
        points = self.convert_pointcloud2_to_json(msg)
        asyncio.run_coroutine_threadsafe(self.broadcast(points), self.loop)

    def convert_pointcloud2_to_json(self, msg):
        points = []
        data = msg.data
        point_step = msg.point_step
        for i in range(0, len(data), point_step):
            x, y, z = struct.unpack_from('fff', data, i)
            points.append({'x': x, 'y': y, 'z': z})
        return json.dumps({'points': points})

    async def start_websocket_server(self):
        async def handler(websocket, path):
            self.websocket_clients.add(websocket)
            try:
                await websocket.wait_closed()
            finally:
                self.websocket_clients.remove(websocket)

        server = await websockets.serve(handler, '0.0.0.0', 8765)
        await server.wait_closed()

    async def broadcast(self, message):
        if self.websocket_clients:
            await asyncio.gather(*[client.send(message) for client in self.websocket_clients])

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudWebSocket()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
