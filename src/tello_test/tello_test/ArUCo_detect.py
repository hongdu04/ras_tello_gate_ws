from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import sys

# ref: https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

#ap = argparse.ArgumentParser()
#ap.add_argument("-i", "--image", required=True, help="path to input image containing ArUCo tag")
#ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="type of ArUCo tag to detect")
#args = vars(ap.parse_args())

ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str, default="DICT_4X4_100", help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
}

desired_aruco_dictionary = "DICT_4X4_100"

print("[INFO] loading image...")
image = cv2.imread("image/markers.png")
image = imutils.resize(image, width=600)

#if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
#	print("[INFO] ArUCo tag of '{}' is not supported".format(desired_aruco_dictionary))
#	sys.exit(0)

if ARUCO_DICT.get(args["type"], None) is None:
	print("[INFO] ArUCo tag of '{}' is not supported".format(args["type"]))
	sys.exit(0)

#print("[INFO] detecting '{}' tags...".format(desired_aruco_dictionary))
#arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dictionary])
#arucoParams = cv2.aruco.DetectorParameters_create()

print("[INFO] detecting '{}' tags...".format(args["type"]))
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters_create()

# (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)

while True:
    frame = vs.read()
    # print(frame)
    frame = imutils.resize(frame, width=1000)

    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)


    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            # draw the bounding box of the ArUCo detection
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
            # compute and draw the center (x, y)-coordinates of the ArUco
            # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            # draw the ArUco marker ID on the image
            cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            print("[INFO] ArUco marker ID: {}".format(markerID))
        # print(frame)
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)
    if key == ord("q"):
        break

        
		# print("[INFO] ArUco marker ID: {}".format(markerID))
		# show the output image
		# cv2.imshow("Image", image)
		# cv2.waitKey(0)
cv2.destroyAllWindows()
vs.stop()