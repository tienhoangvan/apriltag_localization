import apriltag
import cv2
from imutils.video import VideoStream
import math

font = cv2.FONT_HERSHEY_SIMPLEX

# the real distance OX of the qrcode
R_D = 0.097

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
#cap = cv2.VideoCapture(2)
vs = VideoStream(src=2).start()

# define the AprilTags detector options
options = apriltag.DetectorOptions(families="tag16h5")
detector = apriltag.Detector(options)

while True:
      frame = vs.read()
      
      size = frame.shape
      image_center_x = size[1]/2
      image_center_y = size[0]/2
      
      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      
      # detect the AprilTags
      results = detector.detect(gray)
      #print("results: ", results)
      
      cv2.circle(frame, (image_center_x, image_center_y), 5, (255, 0, 0), -1)
      cv2.putText(frame, "C", (image_center_x, image_center_y - 10), font, 0.5, (255, 0, 0), 2)
      
      # loop over the AprilTag detection results
      for r in results:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptO, ptX, ptC, ptY) = r.corners
            ptO = (int(ptO[0]), int(ptO[1]))
            ptX = (int(ptX[0]), int(ptX[1]))
            ptY = (int(ptY[0]), int(ptY[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            
            # draw the bounding box of the AprilTag detection
            cv2.line(frame, ptO, ptX, (0, 255, 0), 2)
            cv2.line(frame, ptX, ptC, (0, 255, 0), 2)
            cv2.line(frame, ptC, ptY, (0, 255, 0), 2)
            cv2.line(frame, ptY, ptO, (0, 255, 0), 2)
      
            # draw the tag family on the image
            cv2.putText(frame,"O", (ptO[0], ptO[1] - 15),font, 0.5, (0, 255, 0), 2)
            cv2.putText(frame,"X", (ptX[0], ptX[1] - 15),font, 0.5, (0, 255, 0), 2) 
            cv2.putText(frame,"Y", (ptY[0] - 15, ptY[1]),font, 0.5, (0, 255, 0), 2)

            # the distance OX of the qrcode in the image
            I_D = math.hypot((ptO[0] - ptX[0]), (ptO[1] - ptX[1]))
            
            Rx = 0
            Ry = 0
            
            # Finding the coordinates of the camera in the real-world domain
            camera_pose_x = Rx + (image_center_y - ptO[1])*R_D/I_D
            camera_pose_y = Ry + (ptO[0] - image_center_x)*R_D/I_D
            
            # Finding the theta angle
            delta_x = ptX[0] - ptO[0]
            delta_y = ptX[1] - ptO[1]
            camera_pose_theta = math.atan2(delta_x, delta_y)
            
            print("Camera Pose: x, y, theta: ", camera_pose_x, camera_pose_y, math.degrees(camera_pose_theta))
            
            
      # show the output image after AprilTag detection             
      cv2.imshow("Apriltag", frame)
      
      if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows() 
cv2.VideoCapture(2).release()