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
        # self.room_direction = {
        #     'top-left': [1.7, 1.7],
        #     'top-right': [1.7, -1.7],
        #     'down-left': [-1.7, 1.7],
        #     'down-right': [-1.7, -1.7]
        # }

        self.room_directions = [
            [1.7, 1.7],
            [-1.7, -1.7],
            [-1.7, 1.7],
            [1.7, -1.7]
        ]
        self.person_location = None

        self.target_time = None
        self.detection_score = None
        self.drone_speed = 0.05
        self.has_initialised = False
        self.drone_turn = radians(90)
        self.count = 0
        self.person_found = False
        # SUBSCRIBERS
        self.image_subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',
            self.image_callback,
            10)

        # PUBLISHERS
        self.twist_publisher = self.create_publisher(
            geometry_msgs.msg.Twist,
            '/drone1/cmd_vel',
            10)

    def image_callback(self, image_message):

        cv_image = self.bridge.imgmsg_to_cv2(
            image_message, desired_encoding='bgr8')

        self.detection_score = self.object_detection.get_detection_image(
            cv_image)

        if not self.person_found:
            self.locate_person()

    def locate_person(self):
        twist = geometry_msgs.msg.Twist()

        if not self.has_initialised:
            self.logger.info(
                '''Drone initialised. Starting via
                moving to top left position.''')

            twist.angular.z = (self.drone_turn * self.drone_speed) / 2
            self.has_initialised = True
        else:
            if self.detection_score[0] < 0.99:
                twist.angular.z = self.drone_turn * self.drone_speed
            else:
                self.person_location = self.room_directions[self.count]
                print('Person Found!')
                self.twist_publisher.publish(geometry_msgs.msg.Twist())
                self.person_found = True
                print(self.person_location)

        if self.person_location is None:
            if self.target_time is None:
                self.target_time = time.time() + 6
            if time.time() >= self.target_time:
                # count increases every time drone passes a corner
                # and person is not found.
                self.count += 1
                print(self.count)
                self.target_time = time.time() + 6
        else:
            print(self.person_location)

        self.twist_publisher.publish(twist)

    def rotate_drone(self):
        """
        Rotates drone within clock-wise direction along the
        Z axis (yaw).
        """

        twist = geometry_msgs.msg.Twist()
        twist.angular.z = self.drone_speed   # Radians per second.
        if self.target_time is None:
            self.logger.info('Timer initialising.')
            self.target_time = time.time() + 21
            self.twist_publisher.publish(twist)

        if time.time() >= self.target_time:
            self.logger.info('Drone is facing top of the room.')
            self.twist_publisher.publish(geometry_msgs.msg.Twist())
            time.sleep(5)
            self.twist_publisher.publish(twist)
            self.target_time = time.time() + 20


rclpy.init()
drone_controller = DroneNode()

rclpy.spin(drone_controller)

drone_controller.destroy_node()
rclpy.shutdown()
