import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

state_names = {
    Transition.TRANSITION_CONFIGURE: "CONFIGURED (Inactive)",
    Transition.TRANSITION_ACTIVATE: "ACTIVE",
    Transition.TRANSITION_DEACTIVATE: "DEACTIVATED (Inactive)",
    Transition.TRANSITION_CLEANUP: "UNCONFIGURED"
}
class LifecycleManagerNode(Node):
    def __init__(self, target_nodes=None):
        super().__init__('lifecycle_manager_node')
        # Add ordered list of nodes to be controlled
        self.target_nodes = target_nodes or ['websocket_node','proximity_sensor_node','safety_control_node','camera_node','driver_node','power_control_node','odometer_node','light_control_node']
        self.lifecycle_clients = {}

        # Created service client for each controlled node
        for node_name in self.target_nodes:
            srv_name = f'/{node_name}/change_state'
            self.lifecycle_clients[node_name] = self.create_client(ChangeState, srv_name)

    async def change_node_state(self, node_name: str, transition_id: int) -> bool:
        """Command node changes"""
        client = self.lifecycle_clients[node_name]

        if not client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(f'Service {client.srv_name} not found!')
            return False

        request = ChangeState.Request()
        request.transition.id = transition_id

        # Send command and wait response
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        state_text = state_names.get(transition_id, f"ID {transition_id}")
        if response and response.success:
            self.get_logger().info(f'Node {node_name} moved to state {state_text} successfully')
            return True
        else:
            self.get_logger().error(f'Node {node_name} state change to state {state_text} failed!')
            return False

    async def startup_robot(self):
        """Startup all nodes in selected order"""
        self.get_logger().info('Starting up nodes')

        # Set node on_configure state (Transition ID 1)
        self.get_logger().info('--- Phase 1: Configuring all nodes ---')
        for node_name in self.target_nodes:
            success = await self.change_node_state(node_name, Transition.TRANSITION_CONFIGURE)
            if not success:
                self.get_logger().error(f'Node configuration prevented due to an error on node {node_name}!')
                return False

         # Set node to on_activate (Transition ID 3 = Activate)
        self.get_logger().info('--- Phase 2: Activating all nodes ---')
        for node_name in self.target_nodes:
            success = await self.change_node_state(node_name, Transition.TRANSITION_ACTIVATE)
            if not success:
                self.get_logger().error(f'Node activation stopped due to an error on node {node_name}!')
                return False
        self.get_logger().info('All nodes are ready and active!')
        return True

def main(args=None):
    rclpy.init(args=args)
    manager = LifecycleManagerNode()

    # Run asynchronously startup sequence
    import asyncio
    asyncio.run(manager.startup_robot())

    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()
