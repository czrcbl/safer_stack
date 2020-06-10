from __future__ import division, print_function
from .outputs import Bbox, BboxList
import cv2
import numpy as np


class Tracker:
    
    def __init__(self, tracker_type):
        tracker_types = ['BOOSTING', 'MIL', 'KCF','TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
        
        self.tracker_type = tracker_type
        
        # Create a tracker based on tracker name
        if tracker_type == tracker_types[0]:
            tracker = cv2.TrackerBoosting_create()
        elif tracker_type == tracker_types[1]: 
            tracker = cv2.TrackerMIL_create()
        elif tracker_type == tracker_types[2]:
            tracker = cv2.TrackerKCF_create()
        elif tracker_type == tracker_types[3]:
            tracker = cv2.TrackerTLD_create()
        elif tracker_type == tracker_types[4]:
            tracker = cv2.TrackerMedianFlow_create()
        elif tracker_type == tracker_types[5]:
            tracker = cv2.TrackerGOTURN_create()
        elif tracker_type == tracker_types[6]:
            tracker = cv2.TrackerMOSSE_create()
        elif tracker_type == tracker_types[7]:
            tracker = cv2.TrackerCSRT_create()
        else:
            tracker = None
            raise ValueError('Incorrect tracker name, available trackers: {}'.format(tracker_types))

        self.tracker = tracker

    def init(self, frame, bbox):
        frame = frame[:, :, ::-1]
        self.currbbox = bbox
        status = self.tracker.init(frame, tuple(bbox.xy_width_height))
        return status

    def update(self, frame):
        status, bbox = self.tracker.update(frame)
        outbb = self.currbbox.copy()
        outbb.bbox = np.array([bbox[0], bbox[1], bbox[0] + bbox[2], bbox[1] + bbox[3]])
        self.currbbox = outbb
        return status, outbb


class MultiTracker:

    def __init__(self, tracker_type='KCF', iou_th=0.5):
        self.trackermap = {}
        self.iou_th = iou_th
        self.tracker_type = tracker_type

    def __len__(self):
        return len(self.trackermap)

    def update(self, frame, bboxlist=None):
        frame = frame[:, :, ::-1]
        out_bboxes = BboxList()
        for _id, tracker in self.trackermap.items():
            status, bbox = tracker.update(frame)
            if status:
                out_bboxes.append(bbox)
      
        if bboxlist is not None:
            for inbbox in bboxlist:
                keep_id = False
                for outbbox in out_bboxes:
                    IOU = inbbox.iou(outbbox)
                    if (IOU > self.iou_th) and (inbbox.class_id == outbbox.class_id):
                        keep_id = True
                        break
                if keep_id:
                    inbbox.id = outbbox.id
                else:
                    new_id = np.random.randint(0, 100)
                    while new_id in self.trackermap.keys():
                        new_id = np.random.randint(0, 100)
                    inbbox.id = new_id
                    
                out_bboxes.append(inbbox)

        nobboxes = out_bboxes.remove_overlap(iou_th=self.iou_th)

        _ids = [bb.id for bb in nobboxes]

        to_keep = {}
        for i, _id in enumerate(_ids):
            if _id not in self.trackermap:
                t = Tracker(self.tracker_type)
                status = t.init(frame, nobboxes[i])
                if status:
                    to_keep[_id] = t
            else:
                to_keep[_id] = self.trackermap[_id]

        self.trackermap = to_keep
        
        return nobboxes

    # def update(self, frame, bboxlist=None):
    #     frame = frame[:, :, ::-1]
    #     to_keep = {}
        
    #     # First keep the bboxes that are found on the new frame
    #     to_keep1 = {}
    #     tkbboxes = BboxList()
    #     for _id, tracker in self.trackermap.items():
    #         status, bbox = tracker.update(frame)
    #         if status:
    #             to_keep1[_id] = tracker
    #             tkbboxes.append(bbox)

    #     # Remove overlap on new bboxes:
    #     tkbboxes = tkbboxes.remove_overlap()
    #     for bb in tkbboxes:
    #         to_keep[bb.id] = to_keep1[bb.id]
        
    #     # if there are new boboxes from the classifier check against
    #     # the bboxes present in the current trackers
    #     if bboxlist is not None:
    #         to_keep2 = {}
    #         for inbbox in bboxlist:
    #             replace = False
    #             ignore = False
    #             for _id, tracker in to_keep.items():
    #                 IOU = inbbox.iou(tracker.currbbox)
    #                 if IOU > self.iou_th:
    #                     if inbbox.score > tracker.currbbox.score:
    #                         ignore = False
    #                     else:
    #                         to_keep2[_id] = tracker
    #                         ignore = True 
    #                     break
    #                 else:
    #                     to_keep2[_id] = tracker

    #             if not ignore:
    #                 new_id = np.random.randint(0, 100)
    #                 while new_id in self.trackermap.keys():
    #                     new_id = np.random.randint(0, 100)
    #                 inbbox.id = new_id
    #                 t = Tracker(self.tracker_type)
    #                 status = t.init(frame, inbbox)
    #                 if status:
    #                     to_keep2[new_id] = t

    #     self.trackermap = to_keep2

    #     out = BboxList()
    #     for t in self.trackermap.values():
    #         out.append(t.currbbox)
    #     return out


# class Tracker:    
    
# 	trackerTypes = ['BOOSTING', 'MIL', 'KCF','TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
#     def __init__(self, trackerType):
#         self.trackerType = trackerType
#         self.multiTracker = cv2.MultiTracker_create()

#     def createTrackerByName(self):

#         trackerType = self.trackerType
        
#         # Create a tracker based on tracker name
#         if trackerType == trackerTypes[0]:
#             tracker = cv2.TrackerBoosting_create()
#         elif trackerType == trackerTypes[1]: 
#             tracker = cv2.TrackerMIL_create()
#         elif trackerType == trackerTypes[2]:
#             tracker = cv2.TrackerKCF_create()
#         elif trackerType == trackerTypes[3]:
#             tracker = cv2.TrackerTLD_create()
#         elif trackerType == trackerTypes[4]:
#             tracker = cv2.TrackerMedianFlow_create()
#         elif trackerType == trackerTypes[5]:
#             tracker = cv2.TrackerGOTURN_create()
#         elif trackerType == trackerTypes[6]:
#             tracker = cv2.TrackerMOSSE_create()
#         elif trackerType == trackerTypes[7]:
#             tracker = cv2.TrackerCSRT_create()
#         else:
#             tracker = None
#             print('Incorrect tracker name')
#             print('Available trackers are:')
#             for t in super(Tracker, self).trackerTypes:
#             print(t)
            
#         return tracker

#     def track(self, bboxlist):
        
    


# class Tracker:
    
#     def __init__(self, window_size=3, iou_th=0.5):
#         self.window_size = window_size
#         self.iou_th = iou_th

#         self.candidates = []
#         self.bboxes = BboxList()

#     def track(self, bboxlist):

#         new_candidates = []
#         to_remove = []
#         bboxes_temp = []
#         for obb in bboxlist:
#             found = 0
#             for c in self.candidates:
#                 cbb = c['bbox']
#                 IOU = cbb.iouc(obb)
#                 if IOU > self.iou_th:
#                     c['count'] += 1
#                     if c['count'] > self.window_size:
#                         bboxes_temp.append(cbb)
#                         to_remove.append(c)
#                     break
#             else:
#                 new_candidates.append({
#                     'bbox': obb,
#                     'count': 1
#                 })
#         for item in to_remove:
#             self.candidates.remove(item)  
#         self.candidates.extend(new_candidates)
            
#         # Remove Bounding Boxes that disappeared
#         for bb in self.bboxes:
#             for obb in bboxlist:
#                 if bb.iouc(obb) > self.iou_th:
#                     break
#             else:
#                 self.bboxes.remove(bb)

#         # Add new bounding boxes:
#         self.bboxes.extend(bboxes_temp)

#         print(self.candidates)
#         return self.bboxes