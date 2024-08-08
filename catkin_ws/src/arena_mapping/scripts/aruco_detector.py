#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import argparse
import cv2
import sys
import math
import os

#Marker  class declaration
class marker:
  def __init__(self, markerID, topLeft, topRight, bottomRight, bottomLeft, center ):
    self.markerID = markerID
    self.topLeft = topLeft
    self.topRight = topRight
    self.bottomRight = bottomRight
    self.bottomLeft = bottomLeft
    self.center = center


#Func to convert from  corners from aruco detection to marker objects
def corners2Marker(corners, ids):
    markersList = []
    for (markerCorner, markerID) in zip(corners, ids):

        # extract the marker corners (which are always returned
        # in top-left, top-right, bottom-right, and bottom-left order)
        corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners

        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        # compute and draw the center (x, y)-coordinates of the
        # ArUco marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        center = (cX, cY)

        markersList.append( marker(markerID, topLeft, topRight, bottomRight, bottomLeft, center ) )

    return markersList

def image_callback(msg):
    global cv_image, bridge
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)


def main():
    global cv_image, bridge
    
    print("Initializing aruco_detector by Diego...")
    bridge = CvBridge()
    
    rospy.Subscriber("/camera/image", Image, image_callback)
    pub = rospy.Publisher('/robotPoseByAruco', PoseStamped, queue_size=10)
    image_pub = rospy.Publisher('/img_map_detection', Image, queue_size=10)
    rospy.init_node('aruco_detector', anonymous=True)
    rate = rospy.Rate(10)

    #Aruco parameters
    #arucoDict = 6 x 6 cells without black border, from id 1  up to 50
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    arucoParams = cv2.aruco.DetectorParameters_create()

    origin_point = (-0.875,0.055) # (x,y) distance from first locIds "map origin"
    locWReal = .38
    locHReal = 1.15
    locIds = [1,2,3,4] #Aruco's ids for references
    isH = False

    cv_no_image = cv2.imread(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + '/assets/no_image.jpg')
    image_message = bridge.cv2_to_imgmsg(cv_no_image , "bgr8")

    if cv_no_image is None:
        rospy.logerr('Image not found')
        return

    while not rospy.is_shutdown():
        pose = PoseStamped();

        # Capture the video frame by frame
        try:
            frame = cv_image
            img_width = frame.shape[1];
            img_height = frame.shape[0];
        except:
            rospy.loginfo("Respawning aruco_detector node...")
            sys.exit()

        # detect ArUco markers in the input frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame,
            arucoDict, parameters=arucoParams)
            # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()

            markersList = corners2Marker(corners, ids);

            # Creating an empty Dictionary
            sqrLoc = {}
            sqrLocHomo = {}

            # loop over the detected ArUCo corners
            for marker in markersList:

                # draw the bounding box of the ArUCo detection
                #cv2.line(frame, marker.topLeft, marker.topRight, (0, 255, 0), 2)
                #cv2.line(frame, marker.topRight, marker.bottomRight, (0, 255, 0), 2)
                #cv2.line(frame, marker.bottomRight, marker.bottomLeft, (0, 255, 0), 2)
                #cv2.line(frame, marker.bottomLeft, marker.topLeft, (0, 255, 0), 2)
                ##draw the center (x, y)-coordinates of the
                #cv2.circle(frame, marker.center, 4, (0, 0, 255), -1)
                ##draw the ArUco marker ID on the frame
                #cv2.putText(frame, str(marker.markerID),
                #    (marker.topLeft[0], marker.topLeft[1] - 15),
                #    cv2.FONT_HERSHEY_SIMPLEX,
                #    0.5, (0, 255, 0), 2)

                if (marker.markerID in  locIds):
                    sqrLoc[marker.markerID] = marker


            if len(sqrLoc) == len( locIds):

            ## aproximacion de la "Localizacion"

                sqrLocHomo[ locIds[0] ] = sqrLoc[ locIds[0] ].center
                sqrLocHomo[ locIds[1] ] = ( sqrLoc[ locIds[1] ].center[0], sqrLocHomo[ locIds[0] ][1]  )

                delta_w = sqrLocHomo[ locIds[1] ][0] - sqrLocHomo[ locIds[0] ][0]
                delta_h = math.ceil( (delta_w * locHReal) / locWReal )

                sqrLocHomo[ locIds[2] ] = (sqrLocHomo[ locIds[1] ][0], int(math.ceil( sqrLocHomo[ locIds[1] ][1] + delta_h)))
                sqrLocHomo[ locIds[3] ] = (sqrLocHomo[ locIds[0] ][0], sqrLocHomo[ locIds[2] ][1])

                # Calculate Homography
                pts_src = np.array([sqrLoc[ locIds[0] ].center,sqrLoc[ locIds[1] ].center,sqrLoc[ locIds[2] ].center,sqrLoc[ locIds[3] ].center])
                pts_dst = np.array([sqrLocHomo[locIds[0]],sqrLocHomo[locIds[1]],sqrLocHomo[locIds[2]],sqrLocHomo[locIds[3]]])
                h, status = cv2.findHomography(pts_src, pts_dst)
                isH = True

                #For debug purpose, draw square  for localization marks
                ##cv2.line(frame, sqrLocHomo[locIds[0]], sqrLocHomo[ locIds[1] ], (224, 0, 1153), 1)
                #cv2.line(frame, sqrLocHomo[locIds[1]], sqrLocHomo[ locIds[2] ], (224, 0, 1153), 1)
                #cv2.line(frame, sqrLocHomo[locIds[2]], sqrLocHomo[ locIds[3] ], (224, 0, 1153), 1)
                #cv2.line(frame, sqrLocHomo[locIds[3]], sqrLocHomo[ locIds[0] ], (224, 0, 1153), 1)

        # Warp source image to destination based on homography
        if isH:
            markersHomoList = []
            robotsList = []
            sqrHomoLoc = {}

            im_orto = cv2.warpPerspective(frame, h, (680,580))
            # im_orto = cv2.warpPerspective(frame, h, (img_width,img_height))

            # detect ArUco markers in the trasformed frame
            (corners, ids, rejected) = cv2.aruco.detectMarkers(im_orto,arucoDict, parameters=arucoParams)
            # verify *at least* one ArUco marker was detected
            if len(corners) > 0:
                # flatten the ArUco IDs list
                ids = ids.flatten()
                markersHomoList = corners2Marker(corners, ids);

                for marker in markersHomoList:

                    # draw the bounding box of the ArUCo detection
                    cv2.line(im_orto, marker.topLeft, marker.topRight, (0, 255, 0), 2)
                    cv2.line(im_orto, marker.topRight, marker.bottomRight, (0, 255, 0), 2)
                    cv2.line(im_orto, marker.bottomRight, marker.bottomLeft, (0, 255, 0), 2)
                    cv2.line(im_orto, marker.bottomLeft, marker.topLeft, (0, 255, 0), 2)
                    # draw the center (x, y)-coordinates of the
                    cv2.circle(im_orto, marker.center, 4, (0, 0, 255), -1)
                    # draw the ArUco marker ID on the frame
                    cv2.putText(im_orto, str(marker.markerID),
                        (marker.topLeft[0], marker.topLeft[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)

                for marker in markersHomoList:
                    if (marker.markerID in  locIds):
                        sqrHomoLoc[marker.markerID] = marker
                    else:
                        robotsList.append(marker)


            if len(sqrHomoLoc) == 4 and len(robotsList) > 0:
                for robot in robotsList:

                    r_xv = robot.center[0]
                    #print(sqrHomoLoc[3].center[0])
                    P4xv = sqrHomoLoc[locIds[3]].center[0]
                    w_r = locWReal
                    w_v = sqrHomoLoc[locIds[2]].center[0] - sqrHomoLoc[locIds[3]].center[0]
                    o_xr = origin_point[0]

                    rP4_xv = r_xv - P4xv
                    r_xr = ( rP4_xv * w_r ) / w_v
                    x = -1 * o_xr + r_xr

                    #point = np.matmul(h, [ [robot.center[0]], [robot.center[1]], [1]]);
                    #cv2.circle(im_orto, (point[0],point[1]), 4, (0, 0, 255), -1)


                    r_yv = robot.center[1]
                    #print(sqrHomoLoc[3].center[0])
                    P1yv = sqrHomoLoc[locIds[0]].center[1]
                    h_r = locHReal
                    h_v = sqrHomoLoc[locIds[3]].center[1] - sqrHomoLoc[locIds[0]].center[1]
                    o_yr = origin_point[1]

                    rP1_yv = r_yv - P1yv
                    r_yr = ( rP1_yv * h_r ) / h_v
                    y = -1 * o_yr + r_yr

                    yaw = math.atan2( robot.topRight[1] - robot.topLeft[1]   ,  robot.topRight[0] - robot.topLeft[0] )
                    #print("Robot id: ",robot.markerID," x: ", x , " y: ", y , " teta: ",yaw )

                    # *Quick fix* Use linear regression to characterize errors in position detection
                    # this *shouldn't* be needed if the rest of the parameters are correct
                    #print('Pre-quick-fix x:', x)
                    #print('Pre-quick-fix y:', y)
                    mx, bx = 1.2372,-0.2772
                    x = mx*x + bx

                    my, by = 1.2217, -0.1211
                    y = my*y + by
                    
                    pose.pose.position.x = y #Inverted axis  for simulator 
                    pose.pose.position.y = x #Inverted axis  for simulator 
                    pose.pose.position.z = 0
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = "map"
                    quaternion = tf.transformations.quaternion_from_euler(0, 0, -(yaw- ( math.pi /2))  ) #Angle for simulator 
                    #type(pose) = geometry_msgs.msg.Pose
                    pose.pose.orientation.x = quaternion[0]
                    pose.pose.orientation.y = quaternion[1]
                    pose.pose.orientation.z = quaternion[2]
                    pose.pose.orientation.w = quaternion[3]

            #cv2.imshow("orto", im_orto)
            image_message = bridge.cv2_to_imgmsg(cv2.resize(im_orto, (640, 480)), "bgr8")
        if not pose.pose.position.x == 0.0 and not pose.pose.position.y == 0.0:
            pub.publish(pose)
        rate.sleep()

        # show the output frame
        #cv2.imshow("Frame", frame)
        image_pub.publish(image_message)
        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
    # do a bit of cleanup
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
