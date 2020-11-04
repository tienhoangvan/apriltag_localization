#!/usr/bin/env python

import rospy
import apriltag
import cv2
from imutils.video import VideoStream
import math
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool

font = cv2.FONT_HERSHEY_SIMPLEX

# the real distance OX of the qrcode
R_D = 0.097

p_81 = [9.45, 6.0, math.pi/2, 81]       # COM
p_82 = [13.5, -1.2, -math.pi/2, 82]     # THUOC
p_83 = [9.45, -27.85, -math.pi/2, 83]   # RAC
p_94 = [-0.5, -1.0, 0, 94]              # SAC

Rooms = [p_94, p_81, p_82, p_83]

def sub_active_signal(msg):
    global activate
    activate = msg.data

def apriltag_localization():

    global activate
    global pub_robot_pose

    # initialize the video stream and allow the camera sensor to warm up
    print("[INFO] starting video stream...")
    vs = VideoStream(src=2).start()
    
    # define the AprilTags detector options
    options = apriltag.DetectorOptions(families="tag16h5")
    detector = apriltag.Detector(options)

    while not rospy.is_shutdown():
        
        frame = vs.read()
        
        size = frame.shape
        image_center_x = size[1]/2
        image_center_y = size[0]/2
        
        Rx = float()
        Ry = float()
        Room = float()
        
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

            # Finding 3D coordinates of AprilTag
            
            tag_data = Rooms[r.tag_id]
            Rx = tag_data[0]
            Ry = tag_data[1]
            Room = tag_data[2]
            
            # the distance OX of the qrcode in the image
            I_D = math.hypot((ptO[0] - ptX[0]), (ptO[1] - ptX[1]))
            
            # Finding the coordinates of the camera in the real-world domain
            camera_pose_x = Rx + (image_center_y - ptO[1])*R_D/I_D
            camera_pose_y = Ry + (ptO[0] - image_center_x)*R_D/I_D
            
            # Finding the theta angle
            delta_x = ptX[0] - ptO[0]
            delta_y = ptX[1] - ptO[1]
            camera_pose_theta = math.atan2(delta_x, delta_y)
            
            # Finding the robot's pose
            robot_pose_x = camera_pose_x - 0.28
            robot_pose_y = camera_pose_y
            
            #print("Camera Pose: x, y, theta: ", camera_pose_x, camera_pose_y, math.degrees(camera_pose_theta))
            print("Robot's Pose: x, y, theta: ", robot_pose_x, robot_pose_y, math.degrees(camera_pose_theta))
        
            pub_robot_pose.publish(Quaternion(robot_pose_x, robot_pose_y, camera_pose_theta, Room))

        # show the output frame
        cv2.imshow("Logitech", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break        

        # if STOP, break from the loop
        if activate == False:
            print("[INFO] AprilTag Localization is shutdown.")
            break
         
    #print("[INFO] cleaning up...")
    cv2.destroyAllWindows()
    vs.stop()
  

if __name__ == '__main__':
    global activate
    global pub_robot_pose
    activate = bool()
    
    try:
        # Init the node, publishers, subcribers
        rospy.init_node('apriltag_localization')
        pub_robot_pose = rospy.Publisher('apriltag_pose', Quaternion, queue_size=10)
        sub_activate = rospy.Subscriber('activate_apriltag_localization', Bool, sub_active_signal)
        
        idle = rospy.Rate(50)
        
        while not rospy.is_shutdown():
            if activate:
                apriltag_localization()
            else:
                #print("Activate: ", activate)
                pass
            idle.sleep()        
        
    except rospy.ROSInterruptException:
        pass