import rclpy
from rclpy.node import Node
from tello_msgs.srv import TelloAction


class ClientAsync(Node):
    def __init__(self):
        super().__init__('client')
        self.client = self.create_client(TelloAction, '/drone1/tello_action')
        while not self.client.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')
        self.req = TelloAction.Request()

    def send_request(self, command):
        self.req.cmd = command
        return self.client.call_async(self.req)


rclpy.init()

client = ClientAsync()
service_return = client.send_request('takeoff')

while rclpy.ok():
    rclpy.spin_once(client)
    if service_return.done():
        response = service_return.result()
        break

client.destroy_node()
rclpy.shutdown()
