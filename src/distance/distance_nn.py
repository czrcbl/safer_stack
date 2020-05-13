#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

from detectron2.config import get_cfg
from detectron2.engine import DefaultPredictor
from detectron2 import model_zoo


class DistanceNN:

    def __init__(self):

        self.predictor = self.load_predictor()

        self.color_img = None
        self.rect_img = None
        self.cloud = None

        self.d = None
        self.crest = None

        self.bridge = CvBridge()
        self.pub_dist = rospy.Publisher('/edge_distance', Float32, queue_size=10)
        self.pub_img = rospy.Publisher('/image_edge_highlighted', Image, queue_size=10)
        
        rospy.Subscriber('/bumblebee2/left/image_rect_color', Image, self.color_img_callback)
        rospy.Subscriber('/bumblebee2/left/image_rect', Image, self.rect_img_callback)
        rospy.Subscriber('/bumblebee2/points2', PointCloud2, self.points2_callback)

        
    def color_img_callback(self, data):
        self.color_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def rect_img_callback(self, data):
        self.rect_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def points2_callback(self, data):
        h = data.height
        w = data.width
    
        coords = np.zeros(shape=(h, w, 3))
        for n, point in enumerate(pc2.read_points(data, skip_nans=False)):
            # print(point[0]) 
            j = n % w
            i = n // w
            coords[i, j, :] = point[:3]
        
        self.cloud = coords

        self.detect_crest()

    def load_predictor(self):

        project_path = os.path.abspath(os.path.realpath(os.path.dirname(os.path.dirname(__file__))))
        model_path = os.path.join(project_path, 'data/0model_final.pth')
        cfg = get_cfg()
        cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1  # only has one class (ballon)
        # cfg.MODEL.WEIGHTS = os.path.join(cfg.OUTPUT_DIR, "model_final.pth")
        cfg.MODEL.WEIGHTS = model_path
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5   # set the testing threshold for this model
        predictor = DefaultPredictor(cfg)

        return predictor

    def predict_mask(self, im):
        outputs = self.predictor(im)
        inst = outputs['instances']
        masks = inst.get('pred_masks')
        mask = masks.sum(axis=0) > 0
        masknp = mask.cpu().numpy()
        return masknp

    def detect_crest(self):

        def generate_contour(mask):
            out = np.zeros(shape=(mask.shape[1],))
            th = mask.shape[0] / 10.0
            cols, rows = np.nonzero(mask.T)
            height = int(np.median(rows))
            cols_prev = 0
            i = 0
            check = np.logical_and(height - th < rows, height + th > rows)
            while i < len(cols):
                c = cols[i]
                r = rows[i]
                ch = check[i]
                if ch:
                    out[cols_prev: c + 1] = r
                    cols_prev = c + 1
                i += 1
                # n = np.sum(c == cols)
                # for j in range(n):
                #     if (height - th < rows[i + j]) and (height + th > rows[i + j]):
                #         out[cols_prev: c + 1] = rows[i + j]
                #         cols_prev = c + 1
                #         i += n
                # else: 
                #     if (height - th < rows[i]) and (height + th > rows[i]):
                #         out[cols_prev: c + 1] = r
                #         cols_prev = c + 1
                #         i += 1
            return out.astype(int)
            # print(np.sum(np.isnan(coords)))
        cloud = self.cloud

        cloud[np.isnan(cloud)] = 30
        depth = np.sqrt((cloud ** 2).sum(axis=-1))
        # print(depth.min(), depth.mean(), depth.max())
        # kernel = np.ones((5,5),np.float32)/25
        # depth = cv2.filter2D(depth,-1,kernel)
        th = 10
        max_dist = 20
        diff_arr = np.abs(depth[1:,:] - depth[:-1,:])
        buff = np.zeros(shape=depth.shape)
        buff[1:, :] = diff_arr
        # mask_arr = np.logical_and(buff > th, depth < max_dist)
        mask_arr = buff > th
        mask_img = mask_arr.astype(np.float32) 
        # print(mask_arr.sum())
        
        ## Process Image
        
        edges = cv2.Canny(self.rect_img, 50, 200)
        
        ## Process o NN:
        mask_nn = self.predict_mask(self.color_img[:,:,::-1])
        interzone = np.logical_and(mask_nn, edges)
        # interzone = np.logical_and(edges, mask_img).astype('int')

        out = generate_contour(interzone)
        self.crest = out
        # print(out)  
        # interzone = edges
        # rows, cols = np.nonzero(interzone)
        # height = int(np.median(rows))
        
        distances = []
        # for i in range(depth.shape[1]):
        #     distances.append(depth[height, i])
        # for d in depth[interzone.astype(bool)]:
        #     distances.append(d)
        for d in depth[out, np.arange(depth.shape[1])]:
            distances.append(d)

        d = min(distances)
        self.d = d

        # print('closest point', min(distances))
        img = np.array(self.color_img)
        img[mask_nn] = (0,0,255)
        cv2.imshow('Segmentation', img)
        cv2.waitKey(3)


    def draw_crest(self):
        if self.crest is None: return
        color_img = np.array(self.color_img)
        color_img[self.crest, np.arange(color_img.shape[1]), :] = np.array([255, 0, 0])
        # self.raw_img_draw = raw_img
        return color_img

    def publish(self):
        
        if (self.d is None) or (self.crest is None): return

        raw_img_draw = self.draw_crest()
        out_img_msg = self.bridge.cv2_to_imgmsg(raw_img_draw)
        self.pub_img.publish(out_img_msg)

        dmsg = Float32()
        dmsg.data = self.d
        self.pub_dist.publish(dmsg)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('DistanceCloud', anonymous=True)

    distnn = DistanceNN()
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # rospy.loginfo('Message sent!')
        distnn.publish()
        rate.sleep()

if __name__ == '__main__':
    listener()