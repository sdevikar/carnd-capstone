from styx_msgs.msg import TrafficLight
import numpy as np
import os
import tensorflow as tf
import rospy
import time
import calendar
import cv2
import yaml

from utils import label_map_util
from utils import visualization_utils as vis_util

class TLClassifier(object):
    VISUALIZE = False

    def __init__(self):
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.safe_load(config_string)
        self.img_width = self.config['camera_info']['image_width']
        self.img_height = self.config['camera_info']['image_height']
        #rospy.loginfo("width: {} height: {}".format(self.img_width, self.img_height))
        self.is_real = self.config['is_site']

        self.detection_graph = None
        self.num_detections = None
        self.detection_boxes = None
        self.detection_scores = None
        self.detection_classes = None
        self.visualize_image = None
        self.label_map = None
        self.category_index = None
        self.MIN_SCORE_THRESHOLD = 0.40
        self.NUM_CLASSES = 4

        CLASSIFIER_BASE = os.path.dirname(os.path.realpath(__file__))
        #GRAPH = 'frozen_inference_graph.pb'
        GRAPH = 'quantized_optimized_inference_graph.pb'

        if self.is_real:
            MODEL_NAME = 'ssd_inception_v2_coco_ud_capstone_real'
            rospy.loginfo("In real site environment...use {}/{}".format(
                MODEL_NAME, GRAPH))
        else:
            MODEL_NAME = 'ssd_inception_v2_coco_ud_capstone_sim'
            rospy.loginfo("In simulator environment...use {}/{}".format(
                MODEL_NAME, GRAPH))

        PATH_TO_CKPT = CLASSIFIER_BASE + '/' + MODEL_NAME + '/' + GRAPH
        PATH_TO_LABELS = CLASSIFIER_BASE + '/label_map.pbtxt'


        ### Load Tensorflow model graph
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        ### Load label map
        self.label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(self.label_map,
            max_num_classes=self.NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)


    def run_inference_for_single_image(self, image):
        self.num_detections = None
        self.detection_boxes = None
        self.detection_scores = None
        self.detection_classes = None

        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                # Get handles to input and output tensors
                ops = tf.get_default_graph().get_operations()
                all_tensor_names = \
                    {output.name for op in ops for output in op.outputs}
                tensor_dict = {}
                for key in [
                    'num_detections', 'detection_boxes', 'detection_scores',
                    'detection_classes'
                ]:
                    tensor_name = key + ':0'
                    if tensor_name in all_tensor_names:
                        tensor_dict[key] = \
                            self.detection_graph.get_tensor_by_name(tensor_name)

                image_tensor = \
                    self.detection_graph.get_tensor_by_name('image_tensor:0')

                ### Run inference
                # Expand dimensions since the model expects
                # images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(image, axis=0)
                output_dict = sess.run(tensor_dict,
                             feed_dict={image_tensor: image_np_expanded})

                # all outputs are float32 numpy arrays,
                # so convert types as appropriate
                self.num_detections = output_dict['num_detections'][0]
                self.detection_boxes = output_dict['detection_boxes'][0]
                self.detection_scores = output_dict['detection_scores'][0]
                self.detection_classes = \
                    output_dict['detection_classes'][0].astype(np.uint8)


    def predict_state(self):
        """Since there can be multiple detections in any one image...
            - Get the detected boxes with scores over minimum threshold.
            - Add up scores for each classification.
            - Normalize by dividing total scores by total number of
              detections with scores over the minimum threshold.
            - Predicted classified state is the state with the highest
              normalized score.
        """
        det_scores = { "Green": 0, "Red": 0, "Yellow": 0, "Unknown": 0}

        det_count = 0
        #rospy.loginfo("num_detections: {}, detection_boxes: {}".format(
        #    self.num_detections, self.detection_boxes.shape[0]
        #))

        for i in range(0, self.num_detections):
            if self.detection_scores is not None:
                det_score = self.detection_scores[i]
                if det_score > self.MIN_SCORE_THRESHOLD:
                    det_state = self.detection_classes[i]
                    det_name = self.category_index[det_state]['name']
                    det_scores[det_name] += det_score
                    det_count += 1

        max_det_score = 0
        max_det_state = "Unknown"
        for key in det_scores.keys():
            # Normalize the scores
            if det_count > 0:
                det_scores[key] /= (det_count * 1.0)
            # See if the normalized score is the highest
            if det_scores[key] > max_det_score:
                max_det_score = det_scores[key]
                max_det_state = key

        rospy.loginfo(det_scores)
        rospy.loginfo("Predicted state: {}   Normalized score: {}".format(
            max_det_state, max_det_score))
        return max_det_state

    def save_visualization(self, state, vis):
        """
        Args:
            state: state name label for the traffic light predicted state
            vis (int):
                - 0 indicates visual with NO bounding boxes, just the
                plain captured image itself.
                - 1 indicates visual includes bounding boxes drawn on
                the image to be saved
        """
        # use timestamp to make unique filenames
        f_name = "tl_{}_{}_{}.jpg".format(calendar.timegm(time.gmtime()),
            vis, state)
        dir = 'data/visual'
        if not os.path.exists(dir):
            os.makedirs(dir)
        pathname = '{}/{}'.format(dir, f_name)
        self.visualize_image = self.visualize_image[:, :, ::-1]
        cv2.imwrite(pathname, self.visualize_image)


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        self.visualize_image = None
        image_np = np.asarray(image, dtype="int32")
        #image_np = np.array(image, dtype="int32").reshape(
        #    (self.img_height, self.img_width, 3)).astype(np.uint8)

        ### Detect traffic lights ###
        start_t = time.time()
        self.run_inference_for_single_image(image_np)
        rospy.loginfo('Inference time: {}s'.format(time.time() - start_t))

        predicted_state = self.predict_state()


        ### For visualization #######################################
        if TLClassifier.VISUALIZE:
            # Save image before applying boxes (Useful for capturing
            # images for augmenting training data)
            self.visualize_image = image_np
            self.save_visualization(predicted_state, 0)

            # Save bounding boxes image
            vis_util.visualize_boxes_and_labels_on_image_array(
                image_np,
                np.squeeze(self.detection_boxes),
                np.squeeze(self.detection_classes).astype(np.int32),
                np.squeeze(self.detection_scores),
                self.category_index,
                use_normalized_coordinates=True,
                min_score_thresh=self.MIN_SCORE_THRESHOLD,
                line_thickness=3)
            self.visualize_image = image_np
            self.save_visualization(predicted_state, 1)
        #############################################################


        if predicted_state == "Red":
            return TrafficLight.RED
        elif predicted_state == "Yellow":
            return TrafficLight.YELLOW
        elif predicted_state == "Green":
            return TrafficLight.GREEN

        return TrafficLight.UNKNOWN
