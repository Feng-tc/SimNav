#!/usr/bin/env python
# coding: utf-8

import os
os.environ["PYTORCH_ENABLE_MPS_FALLBACK"] = "1"
os.environ["TORCH_CUDA_ARCH_LIST"] = "8.9"

import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rospy
import rospkg
import yaml

import supervision as sv
from supervision.draw.color import ColorPalette
from ultralytics import YOLO, YOLOE
from ultralytics.utils import LOGGER
LOGGER.setLevel("ERROR")

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tare_planner.msg import DetectionResult


def _default_object_file():
    dev_cfg = Path(__file__).resolve().parent / 'config' / 'objects.yaml'
    if dev_cfg.exists():
        return str(dev_cfg)
    pkg_path = rospkg.RosPack().get_path('semantic_mapping')
    installed_cfg = os.path.join(pkg_path, 'semantic_mapping', 'config', 'objects.yaml')
    if os.path.exists(installed_cfg):
        return installed_cfg
    return os.path.join(pkg_path, 'config', 'objects.yaml')


def is_bbox_red(image_bgr, bbox, min_red_ratio=0.25):
    x1, y1, x2, y2 = [int(v) for v in bbox]
    h, w = image_bgr.shape[:2]
    x1 = max(0, min(x1, w - 1))
    x2 = max(0, min(x2, w))
    y1 = max(0, min(y1, h - 1))
    y2 = max(0, min(y2, h))
    if x2 <= x1 or y2 <= y1:
        return False
    roi = image_bgr[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 80, 80), (10, 255, 255)) | cv2.inRange(hsv, (160, 80, 80), (180, 255, 255))
    return (np.count_nonzero(mask) / float(roi.shape[0] * roi.shape[1])) >= min_red_ratio


class DetectNode:
    def __init__(self, device='cuda'):
        self.CONFIG_DIR = Path(__file__).resolve().parent

        self.detection_stamps = deque(maxlen=10)
        self.rgb_stack = deque(maxlen=10)

        self.platform = rospy.get_param('~platform', 'mecanum_sim')
        self.ANNOTATE = rospy.get_param('~annotate_image', True)
        self.grounding_score_thresh = rospy.get_param('~grounding_score_thresh', 0.3)
        self.device = rospy.get_param('~device', device)
        self.target_object = rospy.get_param('~target_object', 'ball')
        self.use_color_filter = rospy.get_param('~use_color_filter', True)
        self.target_color = rospy.get_param('~target_color', 'red')
        object_file_path = rospy.get_param('~object_file', _default_object_file())

        with open(object_file_path, "r") as file:
            self.object_config = yaml.safe_load(file)
        self.label_template = self.object_config['prompts']
        self.text_prompt_list = []
        for value in self.label_template.values():
            self.text_prompt_list += value['prompts']
        self.text_prompt = " . ".join(self.text_prompt_list) + " ."
        self.text_prompt_list = np.array(self.text_prompt_list)
        print(f"Text prompt: {self.text_prompt}")

        self.grounding_model = YOLOE(self.CONFIG_DIR / "external/yoloe-26x-seg.engine", task="segment")

        if self.ANNOTATE:
            self.box_annotator = sv.BoxAnnotator(color=ColorPalette.DEFAULT)
            self.label_annotator = sv.LabelAnnotator(
                color=ColorPalette.DEFAULT,
                text_padding=4,
                text_scale=0.5,
                text_position=sv.Position.TOP_LEFT,
                color_lookup=sv.ColorLookup.INDEX,
                smart_position=True,
            )
            self.mask_annotator = sv.MaskAnnotator(color=ColorPalette.DEFAULT)
            self.ANNOTATE_OUT_DIR = os.path.join('output/debug_mapper', 'annotated_3d_in_loop_detection')
            self.IMAGE_DIR = os.path.join('output/debug_mapper', 'image_gates')
            self.VIEWPOINT_IMAGE_DIR = os.path.join(os.path.dirname(__file__), 'output/viewpoint_images')
            if os.path.exists(self.ANNOTATE_OUT_DIR):
                os.system(f"rm -r {self.ANNOTATE_OUT_DIR}")
            os.makedirs(self.ANNOTATE_OUT_DIR, exist_ok=True)

        self.bridge = CvBridge()

        self.rgb_sub = rospy.Subscriber('/camera/image', Image, self.image_callback, queue_size=10)
        self.annotated_image_pub = rospy.Publisher('/annotated_image_detection', Image, queue_size=10)
        self.detection_result_pub = rospy.Publisher('/detection_result', DetectionResult, queue_size=50)

        self.call_back_time_stamp = time.time()
        self.log_info('Detection node has been started.')

    def log_info(self, msg):
        rospy.loginfo(msg)

    def inference(self, cv_image):
        image = cv_image[:, :, ::-1]
        start_time = time.time()
        results = self.grounding_model.track(
            image,
            imgsz=(640, 1920),
            half=True,
            conf=self.grounding_score_thresh,
            persist=True,
            tracker=self.CONFIG_DIR / "config/botsort.yaml",
        )
        time1 = time.time()
        boxes = results[0].boxes

        if boxes.id is None:
            self.log_info("No track IDs found in the results.")
            return {
                "bboxes": np.empty((0, 4), dtype=float),
                "labels": np.array([], dtype=str),
                "confidences": np.array([], dtype=float),
                "ids": np.array([], dtype=int)
            }

        bboxes = boxes.xyxy.cpu().numpy()
        confidences = boxes.conf.cpu().numpy()
        class_names = boxes.cls.cpu().numpy()
        class_names = self.text_prompt_list[class_names.astype(int)]
        ids = boxes.id.int().cpu().numpy()

        return {
            "bboxes": bboxes,
            "labels": class_names,
            "confidences": confidences,
            'ids': ids
        }

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        det_stamp = msg.header.stamp.to_sec()
        self.detection_processing(cv_image, det_stamp)

    def detection_processing(self, image, detection_stamp):
        start_time = time.time()
        self.call_back_time_stamp = start_time

        detections = self.inference(image)

        if self.use_color_filter and self.target_color == 'red' and len(detections['bboxes']) > 0:
            keep = []
            for i, label in enumerate(detections['labels']):
                if label == self.target_object and not is_bbox_red(image, detections['bboxes'][i]):
                    continue
                keep.append(i)
            if len(keep) == 0:
                detections = {
                    "bboxes": np.empty((0, 4), dtype=float),
                    "labels": np.array([], dtype=str),
                    "confidences": np.array([], dtype=float),
                    "ids": np.array([], dtype=int),
                }
            else:
                keep = np.array(keep)
                detections = {
                    "bboxes": detections['bboxes'][keep],
                    "labels": detections['labels'][keep],
                    "confidences": detections['confidences'][keep],
                    "ids": detections['ids'][keep],
                }

        detection_time = time.time()

        image_anno = image.copy()
        if self.ANNOTATE:
            bboxes = detections['bboxes']
            labels = detections['labels']
            obj_ids = detections['ids']

            if len(bboxes) > 0:
                class_ids = np.array(list(range(len(labels))))
                annotation_labels = [
                    f"{class_name} {id_}"
                    for class_name, id_ in zip(labels, obj_ids)
                ]
                detections_ = sv.Detections(xyxy=bboxes, class_id=class_ids)
                self.box_annotator.annotate(scene=image_anno, detections=detections_)
                self.label_annotator.annotate(
                    scene=image_anno, detections=detections_, labels=annotation_labels)

        self.publish_detection_results(detections, detection_stamp, image, image_anno)

    def publish_detection_results(self, detections_tracked, detection_stamp, image, image_anno):
        detection_result_msg = DetectionResult()
        detection_result_msg.header.stamp = rospy.Time.from_sec(detection_stamp)
        detection_result_msg.header.frame_id = 'map'

        for i in range(len(detections_tracked['ids'])):
            detection_result_msg.track_id.append(detections_tracked['ids'][i])
            detection_result_msg.x1.append(detections_tracked['bboxes'][i][0])
            detection_result_msg.y1.append(detections_tracked['bboxes'][i][1])
            detection_result_msg.x2.append(detections_tracked['bboxes'][i][2])
            detection_result_msg.y2.append(detections_tracked['bboxes'][i][3])
            detection_result_msg.label.append(detections_tracked['labels'][i])
            detection_result_msg.confidence.append(detections_tracked['confidences'][i])

        detection_result_msg.image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.detection_result_pub.publish(detection_result_msg)

        annotated_image_msg = self.bridge.cv2_to_imgmsg(image_anno, encoding='bgr8')
        annotated_image_msg.header.stamp = rospy.Time.from_sec(detection_stamp)
        annotated_image_msg.header.frame_id = 'map'
        self.annotated_image_pub.publish(annotated_image_msg)


def main():
    rospy.init_node('detection_node')
    DetectNode()
    rospy.spin()


if __name__ == "__main__":
    main()
