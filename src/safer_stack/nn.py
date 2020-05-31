from __future__ import absolute_import, division, print_function
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
from .outputs import SegList, SegInstance, BboxList
from .transforms import SSDDefaultTransform, FasterRCNNDefaultTransform


class BasePredictor(object):

    def __init__(self, backend, ctx=mx.gpu()):

        # if backend not in Segmenter.all_models:
        #     raise ValueError('backend {} no available, avaliabe backends: {}'.format(backend, Segmenter.all_models))

        net = model_zoo.get_model(backend, pretrained=True, ctx=ctx)
        self.classnames = net.classes
        self.ctx = ctx
        self.predictor = net

    def __call__(self, im):
        pred = self.predictor(im)
        return pred

    def preprocess(self, im):
        im, npim = self.transform(nd.array(im))
        im = im.expand_dims(axis=0).as_in_context(self.ctx)
        return im, npim

    def postprocess(self, data, th):
        """"
        
        """
        data = [d.as_in_context(mx.cpu()) for d in data]
        data = [d.asnumpy() for d in data]
        data = [d.squeeze() for d in data]
        data = [d.squeeze() for d in data]
        scores = data[1]
        idxs = scores >= th
        out = []
        for d in data:
            if len(d.shape) == 1:
                out.append(d[idxs])
            elif len(d.shape) == 2:
                out.append(d[idxs, :])
            elif len(d.shape) == 3:
                out.append(d[idxs, :, :])
            else:
                raise ValueError('Wrong shape {} for model output.'.format(d.shape))
        
        return out

    def detect(self, img, th=0.5):
        img, npim = self.preprocess(img)
        pred = self(img)
        return self.postprocess(pred, th), npim


class Segmenter(BasePredictor):
   
    available_models = {
        'mask_rcnn_resnet18_v1b_coco': FasterRCNNDefaultTransform(short=600, max_size=1000)
    }   
    
    def __init__(self, backend='mask_rcnn_resnet18_v1b_coco', ctx=mx.gpu()):
        if backend not in Segmenter.available_models.keys():
            raise ValueError('Backend {} no available, avaliabe backends: {}'.format(backend, Segmenter.available_models.keys()))
        self.transform = Segmenter.available_models[backend]
        super(Segmenter, self).__init__(backend=backend, ctx=ctx)

    def detect(self, img, th=0.5, keep_size=True):
        (ids, scores, bboxes, masks), npim = super(Segmenter, self).detect(img, th)
        seglist = SegList.from_arrays(self.classnames, ids, scores, masks)
        bboxlist = BboxList.from_arrays(ids, scores, bboxes, class_names = self.classnames)

        if keep_size:
            target_size=img.shape[:2]
            orig_size = npim.shape[:2]
            bboxlist, npim = bboxlist.resize(orig_size, target_size), img

        return  seglist, bboxlist, npim


class Detector(BasePredictor):
    
    available_models = {
        'ssd_512_mobilenet1.0_coco': SSDDefaultTransform(width=512, height=512),
        'ssd_512_resnet50_v1_coco': SSDDefaultTransform(width=512, height=512),
        'faster_rcnn_resnet50_v1b_coco': FasterRCNNDefaultTransform(short=600, max_size=1000)
    }

    def __init__(self, backend='faster_rcnn_resnet50_v1b_coco', ctx=mx.gpu()):
        if backend not in Detector.available_models.keys():
            raise ValueError('Backend {} no available, available backends: {}'.format(backend, Detector.available_models.keys()))
        self.transform = Detector.available_models[backend]
        super(Detector, self).__init__(backend=backend, ctx=ctx)

    def detect(self, img, th=0.5, keep_size=True):
        
        (ids, scores, bboxes), npim = super(Detector, self).detect(img, th)
        bboxlist = BboxList.from_arrays(ids, scores, bboxes, class_names = self.classnames)
        
        if keep_size:
            target_size = img.shape[:2]
            orig_size = npim.shape[:2]
            return bboxlist.resize(orig_size, target_size), img
        else:
            return bboxlist, npim



