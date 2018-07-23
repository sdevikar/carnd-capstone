from styx_msgs.msg import TrafficLight
import numpy as np
import os
import six.moves.urllib as urllib
import tarfile
import tensorflow as tf
import zipfile
import cv2
import rospy

class TLClassifier(object):
    def __init__(self):
        self.detection_graph = None
        self.detection_boxes = None
        self.detection_scores = None
        self.detection_classes = None
        self.visualize_image = None

        ## color values from the traffic light
        self.r = None
        self.b = None
        self.g = None

        MODEL_NAME = 'rfcn_resnet101_coco_2017_11_08'
        PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'
        MODEL_FILE = "{}.tar.gz".format(MODEL_NAME)
        DOWNLOAD_BASE = \
            'http://download.tensorflow.org/models/object_detection/'

        ### Download the model file if not already there
        if not os.path.exists(MODEL_FILE):
            print("Downloading {}".format(DOWNLOAD_BASE + MODEL_FILE))
            opener = urllib.request.URLopener()
            opener.retrieve(DOWNLOAD_BASE + MODEL_FILE, MODEL_FILE)
            tar_file = tarfile.open(MODEL_FILE)
            for file in tar_file.getmembers():
                file_name = os.path.basename(file.name)
                if 'frozen_inference_graph.pb' in file_name:
                    tar_file.extract(file, os.getcwd())

        ### Load frozen Tensorflow model
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

    """
    def load_image_into_numpy_array(self, image):
        (im_width, im_height) = image.size
        return np.array(
            image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)
    """

    def is_yellow(self):
        return self.r > 200 and self.g > 200 and self.b < 80

    def is_green(self):
        return self.r < 70 and self.g > 120 and self.b < 90

    def is_red(self):
        return self.r > 120 and self.g < 50 and self.b < 50

    def run_inference_for_single_image(self, image):
        with self.detection_graph.as_default():
            with tf.Session() as sess:
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
                            tf.get_default_graph().get_tensor_by_name(tensor_name)

                image_tensor = \
                    tf.get_default_graph().get_tensor_by_name('image_tensor:0')

                # Run inference
                output_dict = sess.run(tensor_dict,
                             feed_dict={image_tensor: np.expand_dims(image, 0)})

                # all outputs are float32 numpy arrays,
                # so convert types as appropriate
                self.detection_boxes = output_dict['detection_boxes'][0]
                self.detection_scores = output_dict['detection_scores'][0]
                self.detection_classes = \
                    output_dict['detection_classes'][0].astype(np.uint8)


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #image_np = self.load_image_into_numpy_array(image)
        image_np = np.asarray(image, dtype="int32")
        #rospy.loginfo("get_classification() image_np.shape: {}".format(
        #    image_np.shape))
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)

        ### Detect traffic lights ###
        self.run_inference_for_single_image(image_np)

        # Get the detected box with the highest score over minimum threshold
        MIN_THRESHOLD = 0.50
        CLASS_TRAFFIC_LIGHT = 10

        max_score_indx = np.argmax(self.detection_scores)
        score = self.detection_scores[max_score_indx]
        clazz = self.detection_classes[max_score_indx]

        if score > MIN_THRESHOLD and clazz == CLASS_TRAFFIC_LIGHT:
            box = self.detection_boxes[max_score_indx]
            #print("box  : {}".format(box))
            #print("class: {}".format(clazz))
            #print("score: {}".format(score))

            ### Determine color of the light ###
            # box array ([ymin, xmin, ymax, xmax])
            height = image_np.shape[0]
            width = image_np.shape[1]
            # box values are normalized so need to denormalize them
            # to get the right coordinates from the image
            TRIM = 2
            ymin = int(box[0] * height) + TRIM
            xmin = int(box[1] * width) + TRIM
            ymax = int(box[2] * height) - TRIM
            xmax = int(box[3] * width) - TRIM



            tl_box = image_np[ymin+TRIM:ymax-TRIM, xmin+TRIM:xmax-TRIM]
            tl_box = tl_box.astype(np.uint8)

            tl_box_copy = tl_box.copy()
            gray_tl = cv2.cvtColor(tl_box, cv2.COLOR_RGB2GRAY)

            ### Introduce a GaussianBlur to remove high frequency noise
            BLUR_RADIUS = 41
            gray_tl = cv2.GaussianBlur(gray_tl, (BLUR_RADIUS, BLUR_RADIUS), 0)
            (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray_tl)

            ### Save visualzed image of detected traffic light,
            ### highlighting the region around the brightest pixel
            tl_copy = tl_box_copy.copy()
            cv2.circle(tl_copy, maxLoc, 15, (0, 0, 255), 2)
            self.visualize_image = tl_copy

            # coordinates of the brightest pixel (relative the the box image)
            y, x = maxLoc
            #print(tl_copy[x, y, :])

            color = tl_copy[x, y, :]
            self.r = color[0]
            self.g = color[1]
            self.b = color[2]
            rospy.loginfo("light color rgb: {}".format(color))

            """
            if self.is_red():
                return TrafficLight.RED
            elif self.is_yellow():
                return TrafficLight.YELLOW
            elif self.is_green():
                return TrafficLight.GREEN
            """
            box_height = ymax - ymin
            top_third = box_height / 3
            middle = box_height / 3 * 2
            if y < top_third:
                return TrafficLight.RED
            elif y < middle and y >= top_third:
                return TrafficLight.YELLOW
            elif y < box_height and y >= middle:
                return TrafficLight.GREEN

        return TrafficLight.UNKNOWN
