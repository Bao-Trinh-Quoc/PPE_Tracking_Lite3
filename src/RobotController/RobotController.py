import cv2

from . FpsCounter import FpsCounter
from . YoloWrapper import YoloWrapper
from . ROSTransfer import TransferConstants

kUseRos1Transfer = False

if kUseRos1Transfer:
    from . ROSTransfer import ROS1Transfer
else:
    from . ROSTransfer import ROS2Transfer

kDefaultTrackId = 0
kDefaultTrackMode = False

class RobotController(object):
    def __init__(self):
        self.fps_counter = FpsCounter.FpsCounter()
        self.yolo_wrapper = YoloWrapper.YoloWrapper()
        if kUseRos1Transfer:
            self.ros1_transfer = ROS1Transfer.ROS1Transfer()
        else:
            self.ros2_transfer = ROS2Transfer.ROS2Transfer()
    
        self.is_tracking = kDefaultTrackMode
        self.target_id = kDefaultTrackId

        self.last_linear_velocity = 0.0

    def SetTargetId(self, id):
        self.target_id = id

    def GetTargetId(self):
        return self.target_id

    def SetIsTracking(self, state):
        self.is_tracking = state

    def GetIsTracking(self):
        return self.is_tracking

    def FindTarget(self, boxes):
        for box in boxes:
            if box.id != None:
                if box.id.item() == self.GetTargetId():
                    return box
        return None

    def TrackAndDraw(self, frame, box):
        shape = frame.shape
        frame = cv2.UMat(frame)

        self.fps_counter.Count()

        if(box != None):        

            x1 = int(box.xyxy[0][0].item())
            y1 = int(box.xyxy[0][1].item())
            x2 = int(box.xyxy[0][2].item())
            y2 = int(box.xyxy[0][3].item())

            center=((x1+x2)//2, (y1+y2)//2)
            angle=(center[0]/shape[1]-0.5)*2        # range -1 to 1
            radian_velocity = -angle*TransferConstants.kMaxRadianVelocity
            cv2.circle(frame,center,2,[0,0,255],-1) # 画出选框中心点
            top=y1/shape[0]

            kp_linear_x = 4
            kd_linear_x = 0.3
            
            kBackupSpeed = -0.30            # Slow backup speed (tune: more negative = faster backup)
            kTooCloseTop = 0.05             # Below this = too close, back up
            kDesiredTop = 0.10              # Below this = slow down to stop
            kDeadZoneTop = 0.15             # Between kDesiredTop and this = dead zone (do nothing)
            # Above kDeadZoneTop            = follow forward

            print(f"top={top:.3f}")
            if top < kTooCloseTop:
                # Zone 1: Too close — back up slowly
                linear_velocity = kBackupSpeed
            elif top < kDesiredTop:
                # Zone 2: Gradually slow down to stop
                ratio = (top - kTooCloseTop) / (kDesiredTop - kTooCloseTop)
                linear_velocity = kBackupSpeed * (1.0 - ratio)
            elif top < kDeadZoneTop:
                # Zone 3: Dead zone — robot stays still, no oscillation
                linear_velocity = 0.0
            elif top < 0.5:
                # Zone 4: Normal following — proportional speed
                linear_velocity = (top - kDeadZoneTop) * kp_linear_x - self.last_linear_velocity * kd_linear_x
            else:
                linear_velocity = TransferConstants.kMaxLinerVelocity
            
            cv2.putText(frame,f"Tracking Vest ID: {self.GetTargetId()}",(20,250), cv2.FONT_HERSHEY_PLAIN, 2, [0,255,0], 2)
#              cv2.putText(frame,"Autonomous Mode Active",(20,300), cv2.FONT_HERSHEY_PLAIN, 2, [0,255,0], 2)

            if kUseRos1Transfer:
                self.ros1_transfer.SendCmdVel(linear_velocity, radian_velocity)
            else:
                self.ros2_transfer.SendCmdVel(linear_velocity, radian_velocity)
            self.last_linear_velocity = linear_velocity
            return frame
        else:
            #cv2.putText(frame,"Miss Person",(20,250), cv2.FONT_HERSHEY_PLAIN, 2, [0,0,255], 3)
            return frame

    def NonTrackAndDraw(self, frame):
        frame = cv2.UMat(frame)

        cv2.putText(frame,"Stop",(20,250), cv2.FONT_HERSHEY_PLAIN, 2, [0,0,255], 3)
        #cv2.putText(frame,"Searching for Safety Vest...",(20,300), cv2.FONT_HERSHEY_PLAIN, 2, [255,0,0], 2)
        
        # Ensure robot is stopped when not tracking
        if kUseRos1Transfer:
            self.ros1_transfer.SendCmdVel(0.0, 0.0)
        else:
            self.ros2_transfer.SendCmdVel(0.0, 0.0)
            
        return frame

    def Run(self, frame):
        results = self.yolo_wrapper.Track(frame)
        if(len(results)>0):
            # Render labels dynamically with YOLO plot
            plotted_frame = results[0].plot()
            boxes = results[0].boxes
            
            # Autonomous Target Acquisition
            if not self.GetIsTracking():
                for box in boxes:
                    # Target class 16 (safety-vest)
                    if box.id is not None and int(box.cls[0].item()) == 16:
                        self.SetTargetId(int(box.id[0].item()))
                        self.SetIsTracking(True)
                        break
            
            # Target Following
            if self.GetIsTracking():
                box = self.FindTarget(boxes)
                # If target lost, reset and go back to searching mode
                if box is None:
                    self.SetIsTracking(False)
                    self.SetTargetId(0)
                    return self.NonTrackAndDraw(plotted_frame)
                    
                return self.TrackAndDraw(plotted_frame, box)
            else:
                return self.NonTrackAndDraw(plotted_frame)
        
        return frame






