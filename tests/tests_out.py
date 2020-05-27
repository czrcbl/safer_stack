import os
from os.path import join as pjoin
import sys
import unittest
import matplotlib.pyplot as plt
import cv2
import numpy as np
from safer_stack.outputs import Bbox, BboxList, SegInstance, SegList
from safer_stack.tracking import Tracker

proj_path = os.path.dirname(
    os.path.dirname(
        os.path.abspath(
            os.path.realpath(__file__)
        )
    )
)
test_path = pjoin(proj_path, 'tests')

class TestOutputs(unittest.TestCase):

    def setUp(self):
        self.im = plt.imread(pjoin(test_path, 'sample_img.jpg'))
        self.sample_bb = Bbox([100, 100, 200, 200] , 0, 0.7, 'class0')
    
    def test_draw(self):
        dimg = self.sample_bb.draw(self.im)

    def test_crop(self):
        img = np.zeros((300, 300, 3), dtype=np.float)
        cimg = self.sample_bb.crop_image(img)
        self.assertEqual(cimg.shape, (100, 100, 3))

    def test_iou(self):
        other = Bbox([100, 100, 200, 200] , 0, 0.7, 'class0')
        IOU = self.sample_bb.iou(other)
        self.assertEqual(IOU, 1.0)

        other = Bbox([100, 100, 150, 150] , 0, 0.7, 'class0')
        IOU = self.sample_bb.iou(other)
        self.assertEqual(IOU, 0.25)

    def test_tracker(self):

        t = Tracker()
        bb = self.sample_bb
        bblist = BboxList()
        bblist.append(bb)
        # print(bblist)
        for i in range(5):
            out = t.track(bblist)
            if i in [0, 1, 2]:
                self.assertEqual(len(out), 0)
            else:
                self.assertEqual(len(out), 1)

        # seg, info = self.predictor.panoptic(img)
        # data = self.predictor(self.im)
        # print(data)


    # def test_upper(self):
    #     self.assertEqual('foo'.upper(), 'FOO')

    # def test_isupper(self):
    #     self.assertTrue('FOO'.isupper())
    #     self.assertFalse('Foo'.isupper())

    # def test_split(self):
    #     s = 'hello world'
    #     self.assertEqual(s.split(), ['hello', 'world'])
    #     # check that s.split fails when the separator is not a string
    #     with self.assertRaises(TypeError):
    #         s.split(2)

if __name__ == '__main__':
    unittest.main()