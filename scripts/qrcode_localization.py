#!/usr/bin/env python

import rospy
import math
import cv2
import numpy as np
from imutils.video import VideoStream
import imutils
from pyzbar import pyzbar
import re
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool


BLUR_VALUE = 3
SQUARE_TOLERANCE = 0.15
AREA_TOLERANCE = 0.15
DISTANCE_TOLERANCE = 0.25
WARP_DIM = 300
SMALL_DIM = 29


def count_children(hierarchy, parent, inner=False):
    if parent == -1:
        return 0
    elif not inner:
        return count_children(hierarchy, hierarchy[parent][2], True)
    return 1 + count_children(hierarchy, hierarchy[parent][0], True) + count_children(hierarchy, hierarchy[parent][2], True)

def has_square_parent(hierarchy, squares, parent):
    if hierarchy[parent][3] == -1:
        return False
    if hierarchy[parent][3] in squares:
        return True
    return has_square_parent(hierarchy, squares, hierarchy[parent][3])

def get_center(c):
    m = cv2.moments(c)
    return [int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])]

def get_angle(p1, p2):
    x_diff = p2[0] - p1[0]
    y_diff = p2[1] - p1[1]
    return math.degrees(math.atan2(y_diff, x_diff))

def get_midpoint(p1, p2):
    return [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2]

def qrcode_localization():

    global activate
    global pub_robot_pose

    # initialize the video stream and allow the camera sensor to warm up
    #print("[INFO] starting video stream...")
    # vs = VideoStream(src=0).start()
    vs = VideoStream(src=6).start()

    while not rospy.is_shutdown():
        
        frame = vs.read()
        #frame = imutils.resize(frame, width=640)
        
        size = frame.shape
        image_center_x = size[1]/2
        image_center_y = size[0]/2
        
        output = frame.copy()
        
        Rx = float()
        Ry = float()
        Room = float()
        point_O_x = float()
        point_O_y = float()
        qr_center_x = float()
        qr_center_y = float()

        point_O_list = list()

        check_org = bool()
        check_qr_center = bool()
        
        # Remove noise and unnecessary contours from frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, 11, 17, 17)
        gray = cv2.GaussianBlur(gray, (BLUR_VALUE, BLUR_VALUE), 0)
        edged = cv2.Canny(gray, 30, 200)

        _, contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        squares = []
        square_indices = []
        i = 0
        
        for c in contours:
            # Approximate the contour
            peri = cv2.arcLength(c, True)
            area = cv2.contourArea(c)
            approx = cv2.approxPolyDP(c, 0.03 * peri, True)

            # Find all quadrilateral contours
            if len(approx) == 4:
                # Determine if quadrilateral is a square to within SQUARE_TOLERANCE
                if area > 25 and 1 - SQUARE_TOLERANCE < math.fabs((peri / 4) ** 2) / area < 1 + SQUARE_TOLERANCE and count_children(hierarchy[0], i) >= 2 and has_square_parent(hierarchy[0], square_indices, i) is False:
                    squares.append(approx)
                    square_indices.append(i)

            i += 1
        
        # Determine if squares are QR codes
        for square in squares:
            area = cv2.contourArea(square)
            center = get_center(square)
            peri = cv2.arcLength(square, True)

            similar = []
            tiny = []
            for other in squares:
                if square[0][0][0] != other[0][0][0]:
                    # Determine if square is similar to other square within AREA_TOLERANCE
                    if math.fabs(area - cv2.contourArea(other)) / max(area, cv2.contourArea(other)) <= AREA_TOLERANCE:
                        similar.append(other)
                    elif peri / 4 / 2 > cv2.arcLength(other, True) / 4:
                        tiny.append(other)

            if len(similar) >= 2:
                distances = []
                distances_to_contours = {}
                for sim in similar:
                    sim_center = get_center(sim)
                    d = math.hypot(sim_center[0] - center[0], sim_center[1] - center[1])
                    distances.append(d)
                    distances_to_contours[d] = sim
                distances = sorted(distances)
                closest_a = distances[0]
                closest_b = distances[1]

                # Determine if this square is the top left QR code indicator
                if max(closest_a, closest_b) < cv2.arcLength(square, True) * 2.5 and math.fabs(closest_a - closest_b) / max(closest_a, closest_b) <= DISTANCE_TOLERANCE:
                    # Determine placement of other indicators (even if code is rotated)
                    angle_a = get_angle(get_center(distances_to_contours[closest_a]), center)
                    angle_b = get_angle(get_center(distances_to_contours[closest_b]), center)
                    if angle_a < angle_b or (angle_b < -90 and angle_a > 0):
                        east = distances_to_contours[closest_a]
                        south = distances_to_contours[closest_b]
                    else:
                        east = distances_to_contours[closest_b]
                        south = distances_to_contours[closest_a]

                    #point_O = get_midpoint(get_center(east), get_center(south))
                    c_o = get_center(square)
                    point_O_list.append(c_o)
                                
        # find the qrcodes in the frame and decode each of the qrcodes
        frame = output
        qrcodes = pyzbar.decode(frame)
        
        check_org = False
        check_qr_center = False
        count_qrcode = 0
        
        # loop over the detected qrcodes
        for qrcode in qrcodes:
            
            if count_qrcode >= 1:
                break

            # extract the bounding box location of the qrcode
            (x, y, w, h) = qrcode.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        
            # Decode and get information from qrcode
            qrcodeData = qrcode.data.decode("utf-8")
            qrcodeType = qrcode.type
        
            # draw the qrcode data and qrcode type on the image
            text = "{} ({})".format(qrcodeData, qrcodeType)
            cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
            # Get the corners of the qrcode
            points = qrcode.polygon

            qr_center = get_midpoint(points[0], points[2])
            qr_center_x = qr_center[0]
            qr_center_y = qr_center[1]

            distance_check = math.hypot(qr_center_x - points[0].x, qr_center_y - points[0].y)
            
            match = 0
            # Finding the corresponding origin
            for point_O in point_O_list:
                distance = math.hypot(qr_center_x - point_O[0], qr_center_y - point_O[1])
                if abs(distance_check - distance) <= 0.35*distance_check:
                    point_O_x = point_O[0]
                    point_O_y = point_O[1]
                    match = 1
                    break
                else:
                    match = 0
            if match == 0:
                break

            cv2.circle(frame, (point_O_x, point_O_y), 5, (0, 255, 255), -1)
            cv2.putText(frame, "O", (point_O_x, point_O_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            check_org = True

            cv2.circle(output, (qr_center_x, qr_center_y), 5, (0, 255, 255), -1)
            cv2.putText(output, "Q", (qr_center_x, qr_center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            check_qr_center = True
 
            # Number of points in the convex hull
            n = len(points)
        
            # Draw the convext hull and marking the points
            for j in range(0, len(points)):
                cv2.line(frame, points[j], points[ (j+1) % n], (255,0,0), 2)
        
            # Finding 3D coordinates of qrcode
            indexes = [i.start() for i in re.finditer("#", qrcodeData)]
            #indexes = [i.start() for i in re.finditer(",", qrcodeData)]
        
            Room = float(qrcodeData[0 : indexes[0]])
            #Room = 1

            Rx = qrcodeData[indexes[0]+1 : indexes[1]]
            Rx = float(Rx)
            Ry = qrcodeData[indexes[1]+1 : len(qrcodeData)]
            Ry = float(Ry)

            if check_org and check_qr_center:
                # Finding the coordinates x, y of the camera related to qrcode
                # the real distance OQ of the qrcode
                R_D = 0.094
                # the distance OQ of the qrcode in the image
                I_D = math.hypot((qr_center_x - point_O_x), (qr_center_y - point_O_y))
                    
                # Finding the coordinates of the camera in the real-world domain
                camera_pose_x = Rx + (image_center_y - point_O_y)*R_D/I_D
                camera_pose_y = Ry + (point_O_x - image_center_x)*R_D/I_D
                
                # Finding the theta angle
                delta_x = qr_center_x - point_O_x
                delta_y = qr_center_y - point_O_y
                camera_pose_theta = math.atan2(delta_x, delta_y) + math.pi/4

                # Finding the robot's pose
                robot_pose_x = camera_pose_x - 0.3411
                robot_pose_y = camera_pose_y - 0.0035
        
                #print("Camera Pose: x, y, theta: ", camera_pose_x, camera_pose_y, math.degrees(camera_pose_theta))
                #print("Robot's Pose: x, y, theta: ", robot_pose_x, robot_pose_y, math.degrees(camera_pose_theta))
                    
                pub_robot_pose.publish(Quaternion(robot_pose_x, robot_pose_y, camera_pose_theta, Room))
                
        
        cv2.circle(frame, (image_center_x, image_center_y), 5, (255, 0, 0), -1)
        cv2.putText(frame, "C", (image_center_x, image_center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # show the output frame
        cv2.imshow("Logitech", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                  break

        # if STOP, break from the loop
        if activate == False:
            break
         
    # close the output CSV file do a bit of cleanup
    #print("[INFO] cleaning up...")
    cv2.destroyAllWindows()
    vs.stop()
  
def sub_active_signal(msg):
    global activate
    activate = msg.data


if __name__ == '__main__':
    global activate
    global pub_robot_pose
    activate = bool()
    
    try:
        # Init the node, publishers, subcribers
        rospy.init_node('qrcode_localization')
        pub_robot_pose = rospy.Publisher('QR_pose', Quaternion, queue_size=10)
        sub_activate = rospy.Subscriber('activate_qr_localization', Bool, sub_active_signal)
        
        idle = rospy.Rate(50)
        
        while not rospy.is_shutdown():
            if activate:
                qrcode_localization()
            else:
                #print("Activate: ", activate)
                pass
            idle.sleep()        
        
    except rospy.ROSInterruptException:
        pass