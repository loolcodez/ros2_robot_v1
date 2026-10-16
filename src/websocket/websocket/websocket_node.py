import asyncio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import websockets

class WebsocketNode(Node):

    def __init__(self):
        super().__init__("websocket")
        self.port = 6789

        # Create Publisher for command coming from websocket
        self.move_command_publisher = self.create_publisher(String, 'cmd_vel_manual', 10)

        # Create subscriber for battery status
        self.battery_status_subscription = self.create_subscription(String, '/power/battery_status', self.forward_battery_status, 10)

        # Store for active connections
        self.connected_clients = set()

        self.loop = asyncio.get_event_loop()
        self.loop.create_task(self.start_websocket_server())

    async def start_websocket_server(self):
        self.get_logger().info(f"Websocket: Started Websocket on port: {self.port }")
        async with websockets.serve(self.ws_handler, "0.0.0.0", self.port):
            await asyncio.Future()  # Run forever

    async def ws_handler(self, websocket):
        # Handle incoming messages from websocket
        self.connected_clients.add(websocket)
        self.get_logger().info(f'Websocket: New websocket connection. Active connections: {len(self.connected_clients)}')

        try:
            async for json_message in websocket:
                msg = String()
                msg.data = json_message
                self.get_logger().info(f"Websocket: Received message: {json_message}")

                # Publish the command
                self.move_command_publisher.publish(msg)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.connected_clients.remove(websocket)
            self.get_logger().info(f'Websocket: Connection closed. Active connections: {len(self.connected_clients)}')

    def forward_battery_status(self, msg):
        # Handle battery status message coming from another ROS Node
        self.get_logger().info(f"Websocket: Sending battery status: {msg.data}")
        asyncio.run_coroutine_threadsafe(self.broadcast_to_ws(msg.data), self.loop)

    async def broadcast_to_ws(self, text_data):
        # Send response to all connected websocket clients
        if not self.connected_clients:
            return

        tasks = [client.send(text_data) for client in self.connected_clients]
        await asyncio.gather(*tasks, return_exceptions=True)

async def main_async(args=None):
    rclpy.init(args=args)
    node = WebsocketNode()
    if node is not None:
        node.get_logger().info("Websocket: Startup failure - shutting down")

    # Run node asynchronously without locking asyncio-event loop
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.01)

    node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    # Start app in asyncio-loop
    asyncio.run(main_async(args=args))

if __name__ == '__main__':
    main()
