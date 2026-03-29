import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import asyncio
import websockets
import threading


class RosBridge(Node):
    def __init__(self):
        super().__init__('ros_bridge')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.loop = asyncio.new_event_loop()
        self.ws = None
        threading.Thread(target=self._start_ws, daemon=True).start()

    def _start_ws(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._connect())

    async def _connect(self):
        async with websockets.connect('ws://localhost:3001/bridge') as ws:
            self.ws = ws
            await asyncio.Future()  # keep connection open forever

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        success, jpeg = cv2.imencode('.jpg', frame)
        if success and self.ws:
            asyncio.run_coroutine_threadsafe(
                self.ws.send(jpeg.tobytes()),
                self.loop
            )
            



def main(args=None):
    rclpy.init(args=args)
    node = RosBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
