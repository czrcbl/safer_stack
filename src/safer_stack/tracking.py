from __future__ import division, print_function
from .outputs import Bbox, BboxList


class Tracker:
    
    def __init__(self, window_size=3, iou_th=0.5):
        self.window_size = window_size
        self.iou_th = iou_th

        self.candidates = []
        self.bboxes = BboxList()

    def track(self, bboxlist):

        new_candidates = []
        to_remove = []
        bboxes_temp = []
        for obb in bboxlist:
            found = 0
            for c in self.candidates:
                cbb = c['bbox']
                IOU = cbb.iouc(obb)
                if IOU > self.iou_th:
                    c['count'] += 1
                    if c['count'] > self.window_size:
                        bboxes_temp.append(cbb)
                        to_remove.append(c)
                    break
            else:
                new_candidates.append({
                    'bbox': obb,
                    'count': 1
                })
        for item in to_remove:
            self.candidates.remove(item)  
        self.candidates.extend(new_candidates)
            
        # Remove Bounding Boxes that disappeared
        for bb in self.bboxes:
            for obb in bboxlist:
                if bb.iouc(obb) > self.iou_th:
                    break
            else:
                self.bboxes.remove(bb)

        # Add new bounding boxes:
        self.bboxes.extend(bboxes_temp)

        print(self.candidates)
        return self.bboxes