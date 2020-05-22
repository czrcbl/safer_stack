import os
os.environ['MXNET_CUDNN_AUTOTUNE_DEFAULT'] = '0'
import numpy as np
import cv2
import random
import matplotlib.pyplot as plt
import cv2
import mxnet as mx
from mxnet import nd
from gluoncv import model_zoo
from gluoncv import data as gdata
from gluoncv import utils as gutils
from outputs import SegList, SegInstance
    
class Segmenter:

    def __init__(self, ctx=mx.gpu()):
        net = model_zoo.get_model('mask_rcnn_resnet18_v1b_coco', pretrained=True, ctx=ctx)
        self.classnames = net.classes
        self.transform = gdata.transforms.presets.rcnn.transform_test
        self.ctx = ctx
        self.predictor = net

    def preprocess(self, im):
        im, npim = self.transform(nd.array(im))
        im = im.as_in_context(self.ctx)
        return im, npim

    def __call__(self, im):
        # im, npim = self.preprocess(im)
        pred = self.predictor(im)
        return pred

    def instance_seg(self, im, th=0.5):
        ids, scores, bboxes, masks = self(im)
        ids, scores, bboxes, masks = ids.as_in_context(mx.cpu()), scores.as_in_context(mx.cpu()), bboxes.as_in_context(mx.cpu()), masks.as_in_context(mx.cpu())
        ids, scores, bboxes, masks = ids.asnumpy(), scores.asnumpy(), bboxes.asnumpy(), masks.asnumpy()
        ids, scores, bboxes, masks = ids.squeeze(), scores.squeeze(), bboxes.squeeze(), masks.squeeze()
        idxs = scores >= th
        ids, scores, bboxes, masks = ids[idxs], scores[idxs], bboxes[idxs, :], masks[idxs, :, :]
        return SegList.from_arrays(self.classnames, ids, scores, bboxes, masks)



