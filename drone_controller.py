import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import geometry_msgs.msg
from my_msgs.srv import Location
from drone_object_detection import ObjectDetection
from math import radians
import time


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

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
        self.drone_speed = 0.05  # drone speed multiplier.
        self.has_initialised = False

        # Set turning rate to 90 degrees a second.
        self.drone_turn = radians(90)
        self.count = 0

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

        # CLIENTS
        self.location_client = self.create_client(Location, 'person_location')

    def image_callback(self, image_message):

        cv_image = self.bridge.imgmsg_to_cv2(
            image_message, desired_encoding='bgr8')

        self.detection_score = self.object_detection.get_detection_image(
            cv_image)

        if self.person_location is None:
            self.locate_person()

    def locate_person(self):
        twist = geometry_msgs.msg.Twist()

        twist = self.rotate_and_locate_person(twist)

        self.environment_rotation_timer(6)  # 6 seconds.

        self.twist_publisher.publish(twist)

    def rotate_and_locate_person(self, twist):
        if not self.has_initialised:
            self.logger.info(
                '''Drone initialised. Starting via
                moving to top left position.''')

            # Rotate 45 degrees to reach first corner.
            twist.angular.z = (self.drone_turn * self.drone_speed) / 2

            # Set flag to start full rotation.
            self.has_initialised = True
        else:

            # Locate person based on probability threshold value.
            if self.detection_score[0] < 0.99:

                # Rotate around the environment.
                twist.angular.z = self.drone_turn * self.drone_speed
            else:
                # Set location of person.
                self.person_location = self.room_directions[self.count]
                self.logger.info(
                    'Person Found! Location: {}'
                    .format(self.person_location))
                self.location_async_request()

        return twist

    def environment_rotation_timer(self, timer):
        """
        Method to implement a counter based system, where target_time given
        is the amount of time taken to rotate to each corner.
        The speed is calculated by using the distance (radians) and the time
        taken to reach a certain distance.
        """

        # If timer has not been initialised yet.
        if self.target_time is None:
            self.target_time = time.time() + timer
        if time.time() >= self.target_time:

            # Count increases every time drone passes
            # a corner and person is not found.
            self.count += 1
            self.target_time = time.time() + timer

    def location_async_request(self):
        while not self.location_client.wait_for_service(timeout_sec=1.0):
            self.logger.info(
                'Location service not available, waiting again...')

        self.logger.info('Submitting location of person to turtlebot.')
        request = Location.Request()
        request.location_x, request.location_y = self.person_location
        # Submit an Asynchronous request to the location service to invoke
        # the DQN Gazebo services.
        self.location_client.call_async(request)


rclpy.init()
drone_controller = DroneController()

rclpy.spin(drone_controller)

drone_controller.destroy_node()
rclpy.shutdown()
