import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import geometry_msgs.msg
from drone_object_detection import ObjectDetection
from math import radians
import time


class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')

        self.bridge = CvBridge()
        self.object_detection = ObjectDetection(
            pipeline_config='./train_person.config',
            model_path='./person_model/ckpt-3',
            label_map_path='./datasets/label_map.pbtxt')
        self.logger = self.get_logger()

        # Dictionary object to each room corner.
        self.room_direction = {
            'top-left': [1.7, 1.7],
            'top-right': [1.7, -1.7],
            'down-left': [-1.7, 1.7],
            'down-right': [-1.7, -1.7]
        }

        self.drone_turn = radians(90)
        self.current_room_direction = None
        self.has_initialised = False

        # SUBSCRIBERS
        self.subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',
            self.listener_callback,
            10)

        # PUBLISHERS
        self.twist_publisher = self.create_publisher(
            geometry_msgs.msg.Twist,
            '/drone1/cmd_vel',
            10)

    def listener_callback(self, image_message):

        cv_image = self.bridge.imgmsg_to_cv2(
            image_message, desired_encoding='bgr8')

        self.object_detection.get_object_detection_image(
            cv_image)

        self.locate_person()

    def locate_person(self):
        speed = 0.1
        twist = geometry_msgs.msg.Twist()

        if not self.has_initialised:
            self.logger.info(
                '''Drone initialised. Starting via
                moving to top left position.''')

            twist.angular.z = (self.drone_turn*0.1) / 2

            self.current_room_direction = 'top-left'
            self.has_initialised = True
        else:
            self.logger.info(
                f'''Rotating drone to {self.current_room_direction}.''')

            twist.angular.z = self.drone_turn * 0.1

        self.twist_publisher.publish(twist)
        time.sleep(5.0)

        self.twist_publisher.publish(geometry_msgs.msg.Twist())
        time.sleep(5.0)

    def rotate_drone(self):
        """
        Rotates drone within clock-wise direction along the
        Z axis (yaw).
        """
        twist = geometry_msgs.msg.Twist()
        twist.angular.z = 0.1  # Radians per second.
        self.twist_publisher.publish(twist)


rclpy.init()
drone_controller = DroneNode()

rclpy.spin(drone_controller)

drone_controller.destroy_node()
rclpy.shutdown()
