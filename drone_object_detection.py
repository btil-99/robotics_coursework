import tensorflow as tf
import cv2
from object_detection.utils import label_map_util
from object_detection.utils import config_util
from object_detection.builders import model_builder
import numpy as np


def drawcai(img, boxes, classes, scores, category_index):
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


def get_model_detection_function(model):
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

        # Set physical device to GPU.
        physical_devices = tf.config.list_physical_devices('GPU')
        if len(physical_devices) > 0:
            tf.config.experimental.set_memory_growth(physical_devices[0], True)

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

    def get_object_detection_image(self, cv_image):
        """
        Runs object detection model and generates results along with
        visable borders around the detected entity and shows this image.
        Returns object detection results.

        :param cv_image: OpenCV Image in BGR format.
        """

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
        cv2.waitKey(1)

        return predictions_dict
