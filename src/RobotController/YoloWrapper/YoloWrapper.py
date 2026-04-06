from ultralytics import YOLO

class YoloWrapper:
    def __init__(self):
        self.model = YOLO('../model/best.engine')

    def Track(self, img):
        return self.model.track(img, persist=True, classes=[0, 16], conf=0.2)

