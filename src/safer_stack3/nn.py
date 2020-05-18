import detectron2
from detectron2.utils.logger import setup_logger
setup_logger()

# import some common libraries
import numpy as np
import cv2
import random

# import some common detectron2 utilities
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog
import matplotlib.pyplot as plt

class Segmenter:

    def __init__(self):
        cfg = get_cfg()
        self.cfg = cfg
        cfg.merge_from_file(model_zoo.get_config_file("COCO-PanopticSegmentation/panoptic_fpn_R_50_3x.yaml"))
        cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-PanopticSegmentation/panoptic_fpn_R_50_3x.yaml")
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
        self.predictor = DefaultPredictor(cfg)

    def __call__(self, im):
        pred = self.predictor(im)
        return pred

    def panoptic(self, im):
        return self(im)['panoptic_seg']
    
    def semantic(self, im):
        return self(im)['instances']

    def pred_and_draw(self, im, im_format='cv2'):
        panoptic_seg, segments_info = self(im)['panoptic_seg']
        v = Visualizer(im[:, :, ::-1], MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0]), scale=1.2)
        v = v.draw_panoptic_seg_predictions(panoptic_seg.to('cpu'), segments_info)
        im = v.get_image()
        if im_format == 'cv2':
            return im
        else:
            return im[:, :, ::-1]

