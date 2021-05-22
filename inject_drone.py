import rclpy
import transformations
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose


def inject(xml: str, initial_pose: Pose):
    """Create a ROS node, and use it to call the SpawnEntity service"""

    rclpy.init()
    node = rclpy.create_node('inject_node')
    client = node.create_client(SpawnEntity, 'spawn_entity')

    if not client.service_is_ready():
        node.get_logger().info('waiting for spawn_entity service...')
        client.wait_for_service()

    request = SpawnEntity.Request()
    request.xml = xml
    request.initial_pose = initial_pose
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r'
            % future.exception())

    node.destroy_node()
    rclpy.shutdown()


tello_drone_file = open('./tello_1.urdf', 'r')

pose = Pose()
pose.position.x = float(1)
pose.position.y = float(0)
pose.position.z = float(0)
quaternion = transformations.quaternion_from_euler(0, 0, float(0))
pose.orientation.w = quaternion[0]
pose.orientation.x = quaternion[1]
pose.orientation.y = quaternion[2]
pose.orientation.z = quaternion[3]

inject(tello_drone_file.read(), pose)
