import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import geometry_msgs.msg
from tello_msgs.srv import TelloAction
import tensorflow as tf
from object_detection.utils import label_map_util
from object_detection.utils import config_util
from object_detection.builders import model_builder
import numpy as np


def drawcai(self, img, boxes, classes, scores, category_index):
    height = img.shape[0]
    width = img.shape[1]

    detectn = np.sum(scores > 0.2)

    for i in range(detectn):
        ymin = int(boxes[i][0]*height)
        xmin = int(boxes[i][1]*width)
        ymax = int(boxes[i][2]*height)
        xmax = int(boxes[i][3]*width)

        box_color = (255, 128, 0)  # box color
        box_thickness = 2
        cv2.rectangle(
            img,
            (xmin, ymin),
            (xmax, ymax),
            box_color,
            box_thickness)

        label_text = category_index[classes[i]+1]["name"] + \
            " (" + str(int(scores[i]*100)) + "%)"
        label_background_color = (125, 175, 75)
        label_text_color = (255, 255, 255)
        label_size = cv2.getTextSize(
            label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
        label_xmin = int(xmin)
        label_ymin = int(ymin) - label_size[1]
        if (label_ymin < 1):
            label_ymin = 1
        label_xmax = label_xmin + label_size[0]
        label_ymax = label_ymin + label_size[1]
        cv2.rectangle(
            img,
            (label_xmin - 1, label_ymin - 1),
            (label_xmax + 1, label_ymax + 1),
            label_background_color,
            -1)

        cv2.putText(img, label_text, (label_xmin, label_ymax),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, label_text_color, 1)


def get_model_detection_function(self, model):
    """Get a tf.function for detection."""

    @tf.function
    def detect_fn(image):
        """Detect objects in image."""
        image, shapes = model.preprocess(image)
        prediction_dict = model.predict(image, shapes)
        detections = model.postprocess(prediction_dict, shapes)

        return detections, prediction_dict, tf.reshape(shapes, [-1])

    return detect_fn


class ObjectDetection:

    def __init__(self, pipeline_config, model_path, label_map_path):

        # Pass parameters to object attributes.
        self.pipeline_config = pipeline_config
        self.model_path = model_path
        self.label_map_path = label_map_path

        # Load pipeline config and build a detection model
        configs = config_util.get_configs_from_pipeline_file(pipeline_config)
        model_config = configs['model']
        detection_model = model_builder.build(
            model_config=model_config, is_training=True)
        # Restore checkpoint
        ckpt = tf.compat.v2.train.Checkpoint(model=detection_model)
        ckpt.restore(model_path).expect_partial()

        # Get detection function from saved model.
        self.detect_function = get_model_detection_function(detection_model)

        # Loaded saved label map required for correct labelling within image.
        self.label_map = label_map_util.load_labelmap(label_map_path)
        self.categories = label_map_util.convert_label_map_to_categories(
            self.label_map,
            max_num_classes=label_map_util.get_max_label_map_index(
                self.label_map),
            use_display_name=True)
        self.category_index = label_map_util.create_category_index(
            self.categories)

    def get_object_detection_image(self, image_message):
        """
        Runs object detection model and generates results along with
        visable borders around the detected entity and shows this image.
        Returns object detection results.

        :param image_message: Raw ROS image message.
        """
        cv_image = self.bridge.imgmsg_to_cv2(
            image_message, desired_encoding='bgr8')

        rgb_cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        input_tensor = tf.convert_to_tensor(
            np.expand_dims(rgb_cv_image, axis=0), dtype=tf.float32)

        detections, predictions_dict, shapes = self.detect_function(
            input_tensor)

        drawcai(  # Draws shapes onto CV RGB Image.
            cv_image,
            detections['detection_boxes'][0].numpy(),
            detections['detection_classes'][0].numpy().astype(np.uint32),
            detections['detection_scores'][0].numpy(),
            self.category_index)

        cv2.imshow("Drone Camera", cv_image)

        return predictions_dict


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        self.bridge = CvBridge()
        self.object_detection = ObjectDetection(
            pipeline_config='./train_person.config',
            model_path='./person_model/ckpt-3',
            label_map_path='./datasets/label_map.pbtxt'
        )
        self.logger = rclpy.logging.get_logger()

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

        result = self.object_detection.get_object_detection_image(
            image_message
        )

        self.logger.info('Object detection results: %s' % result)

        self.rotate_drone()

    def rotate_drone(self):
        """
        Rotates drone within clock-wise direction along the
        Z axis (yaw).
        """
        twist = geometry_msgs.msg.Twist()
        twist.angular.z = 0.5
        self.pub.publish(twist)


rclpy.init()
drone_controller = DroneController()

rclpy.spin(drone_controller)

drone_controller.destroy_node()
rclpy.shutdown()
