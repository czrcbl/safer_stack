import os
from os.path import join as pjoin
import sys
proj_path = os.path.dirname(
    os.path.dirname(
        os.path.abspath(
            os.path.realpath(__file__)
        )
    )
)
test_path = pjoin(proj_path, 'tests')
src_path = os.path.join(proj_path, 'src')
if src_path not in sys.path:
    sys.path.insert(0, src_path)
import unittest
import matplotlib.pyplot as plt
import cv2
from safer_stack import nn


class TestSegmenter(unittest.TestCase):

    def setUp(self):
        self.predictor = nn.Segmenter()
        self.im = plt.imread(pjoin(test_path, 'sample_img.jpg'))
    def test_call(self):
        pass
        # seg, info = self.predictor.panoptic(img)
        # data = self.predictor(self.im)
        # print(data)

    def test_instance_seg(self):

        print(self.im)
        im, npim = self.predictor.preprocess(self.im)
        seglist = self.predictor.instance_seg(im)
        print(seglist)

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