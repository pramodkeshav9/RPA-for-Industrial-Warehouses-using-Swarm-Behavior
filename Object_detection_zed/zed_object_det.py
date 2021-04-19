#!/usr/bin/env python3
import sys
import numpy as np
import numpy
import pyzed.sl as sl
import cv2
import math
import rospy
from geometry_msgs.msg import Point

def point_publisher(x,y,z):
        pub = rospy.Publisher('chatter', Point, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(1) # 1hz
        msg=Point()
        msg.x = x
        msg.y = y
        msg.z = z
        pub.publish(msg)

#qrCodeDetector = cv2.QRCodeDetector()

def empty(xyz):
    pass

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters", 640,240)
cv2.createTrackbar("Threshold1", "Parameters", 20, 400, empty)
cv2.createTrackbar("Threshold2", "Parameters", 20, 400, empty)

def getContours(img, imgContour):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 3000:
            #cv2.drawContours(imgContour, contour, -1, (255,0,255), 7)
            points = cv2.approxPolyDP(contour, 0.02*cv2.arcLength(contour, True), True)
            numSides = len(points)
            #print(numSides)
            #if numSides == 4:
            x, y, w, h = cv2.boundingRect(points)
            cv2.rectangle(imgContour, (x, y), (x+w, y+h), (0,255,0), 5)
            #print("x:",x,"y:",y)
            #Distance of object

            #Average distance
            i=int(x+(w/2))
            j=int(y+(h/2))
            x_mid=i
            y_mid=j
            dsts = list()
            cnt=8
            t=-1
            while(cnt):
                err, point_cloud_value = point_cloud.get_value(i,j)
                dt =  math.sqrt( point_cloud_value[0] * point_cloud_value[0] + point_cloud_value[1] * point_cloud_value[1] + point_cloud_value[2] * point_cloud_value[2])
                dsts.append(dt)
                if(cnt%2==0):
                    i=int(i+(t*w/70))
                else:
                    j=int(j+(t*h/70))
                cnt=cnt-1
                t=t*-1

            distance= sum(dsts)/len(dsts)
            point_cloud_np = point_cloud.get_data()
            point_cloud_np.dot(tr_np)

            if not np.isnan(distance) and not np.isinf(distance):
                print("Distance to Camera at ({}, {}) (image center): {:1.3} m".format(x, y, distance), end="\r")
                point_publisher(x_mid,y_mid,distance)
            else:
                print("Can't estimate distance at this position.")
                print("Your camera is probably too close to the scene, please move it backwards.\n")
            sys.stdout.flush()

def Click(event,x,y,flags,param):
   # print('@@@@@@@@@@@@@@')
    #global mouseX,mouseY
    mouseX = x
    mouseY = y
    err, point_cloud_value = point_cloud.get_value(mouseX,mouseY)
    X = (float(round(point_cloud_value[0])))
    Y = (float(round(point_cloud_value[1])))
    Z = (float(round(point_cloud_value[2])))
    distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                 point_cloud_value[1] * point_cloud_value[1] +
                                 point_cloud_value[2] * point_cloud_value[2])
    print(distance)
    #print(X,Y,Z,type(X),type(Y),type(Z))


# Create a ZED camera object
zed = sl.Camera()

# Set configuration parameters
input_type = sl.InputType()
#if len(sys.argv) >= 2 :
#input_type.set_from_svo_file(sys.argv[1])
init = sl.InitParameters(input_t=input_type)
init.camera_resolution = sl.RESOLUTION.HD720
init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init.coordinate_units = sl.UNIT.METER

# Open the camera
err = zed.open(init)
if err != sl.ERROR_CODE.SUCCESS :
    print(repr(err))
    zed.close()
    exit(1)

# Set runtime parameters after opening the camera
runtime = sl.RuntimeParameters()
runtime.sensing_mode = sl.SENSING_MODE.STANDARD

# Prepare new image size to retrieve half-resolution images
image_size = zed.get_camera_information().camera_resolution
image_size.width = image_size.width /2
image_size.height = image_size.height /2

# Declare your sl.Mat matrices
image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
depth_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
point_cloud = sl.Mat()

mirror_ref = sl.Transform()
mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
tr_np = mirror_ref.m

key = ' '
while key != 113 :
    err = zed.grab(runtime)
    if err == sl.ERROR_CODE.SUCCESS :
        # Retrieve the left image, depth image in the half-resolution
        zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
        zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, image_size)
        # Retrieve the RGBA point cloud in half resolution
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, image_size)

        # To recover data from sl.Mat to use it with opencv, use the get_data() method
        # It returns a numpy array that can be used as a matrix with opencv
        frame = image_zed.get_data()
        depth_image_ocv = depth_image_zed.get_data()
        frame = cv2.cvtColor(frame,cv2.COLOR_BGRA2BGR)
        #cv2.imshow("Image", frame)
        #cv2.imshow("Depth", depth_image_ocv)
        #_, frame = capture.read()
        #frame = cv2.imread("/home/ubuntu/objects_detect.jpeg")
        # Converts images from BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([0,50,50]) #red
        upper = np.array([10,255,255])
        # Here we are defining range of bluecolor in HSV
    	# This creates a mask of blue coloured
    	# objects found in the frame.
        mask = cv2.inRange(hsv, lower, upper)
  	# The bitwise and of the frame and mask is done so
    	# that only the blue coloured objects are highlighted
    	# and stored in res
        res = cv2.bitwise_and(frame,frame, mask= mask)
        cv2.imshow('frame',frame)
        cv2.imshow('mask',mask)
        cv2.imshow('res',res)
        imgBlur = cv2.GaussianBlur(res, (7,7), 1)
        imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
        threshold1 = cv2.getTrackbarPos("Threshold1", "Parameters")
        threshold2 = cv2.getTrackbarPos("Threshold2", "Parameters")
        imgCanny = cv2.Canny(imgGray, threshold1, threshold2)
        kernel = numpy.ones((5,5))
        imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
        imgContour = res.copy()
        getContours(imgDil, imgContour)
        #cv2.namedWindow("frame")
        #cv2.setMouseCallback("frame",Click)
        cv2.imshow("found", imgContour)
        #QR code detection
        #decodedText, points, _ = qrCodeDetector.detectAndDecode(frame)
        #if points is not None:
        	#nrOfPoints = len(points)
        	#for i in range(nrOfPoints):
                    #nextPointIndex = (i+1) % nrOfPoints
                    #cv2.line(frame, tuple(points[i][0]), tuple(points[nextPointIndex][0]), (255,0,0), 5)
                    #print(decodedText) 
        key = cv2.waitKey(10)

cv2.destroyAllWindows()
zed.close()

print("\nFINISH")

if __name__ == "__main__":
    main()

