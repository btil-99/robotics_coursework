import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import geometry_msgs.msg
from tello_msgs.srv import TelloAction


class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(Image, 
            '/drone1/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        node = rclpy.create_node('teleop_twist_keyboard')
        self.pub = node.create_publisher(geometry_msgs.msg.Twist, '/drone1/cmd_vel', 10)

        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc('m','p','4','v')
        self.out = cv2.VideoWriter('person_top_left.mp4',fourcc, 15.0, (960,720))

    def listener_callback(self, image_message):

        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
        self.out.write(cv_image)
        cv2.imshow("Image window", cv_image)
        

        key = cv2.waitKey(1)

        speed = 0.2
        turn = 0.5

        twist = geometry_msgs.msg.Twist()
        if key == ord('w') or key == ord('W'):
            twist.linear.x = 1 * speed
            twist.linear.y = 0 * speed
            twist.linear.z = 0 * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0 * turn
            self.pub.publish(twist)
        elif key == ord('a') or key == ord('A'):
            twist.linear.x = 0 * speed
            twist.linear.y = 0 * speed
            twist.linear.z = 0 * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 1 * turn
            self.pub.publish(twist)
        elif key == ord('s') or key == ord('S'):
            twist.linear.x = -1 * speed
            twist.linear.y = 0 * speed
            twist.linear.z = 0 * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0 * turn
            self.pub.publish(twist)
        elif key == ord('d') or key == ord('D'):
            twist.linear.x = 0 * speed
            twist.linear.y = 0 * speed
            twist.linear.z = 0 * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = -1 * turn
            self.pub.publish(twist)
        elif key == ord(' '):
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
        elif key == ord('u') or key == ord('U'):
            twist.linear.x = 0 * speed
            twist.linear.y = 0 * speed
            twist.linear.z = 1 * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0 * turn
            self.pub.publish(twist)
        elif key == ord('i') or key == ord('I'):
            twist.linear.x = 0 * speed
            twist.linear.y = 0 * speed
            twist.linear.z = -1 * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0 * turn
            self.pub.publish(twist)
                
        self.pub.publish(twist)


rclpy.init()
subscriber = Subscriber()

rclpy.spin(subscriber)

subscriber.destroy_node()
rclpy.shutdown()

