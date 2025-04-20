from imutils.video import VideoStream
import imutils
import time
import cv2

# the correct arucos are to be obtained from the identifiers in the objects
ARUCO_OBJECTS = [20,21] # the object markers
ARUCO_PLANE = [23,24,25,29] # the table surface
ARUCO_POSITIONS = [26,28,30,31] # the reference positions for the objects 
ARUCO_ROBOT = 22 # the robot arm

DISTORTION_COEFFICIENT = 0.18#1 # how much is the aruco distorted due to tall objects being far from the center of the image

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

print("[INFO] starting video stream...")
vs = cv2.VideoCapture(2)#VideoStream(src=0).start()
time.sleep(2.0) # warm up delay

# Check if the camera was successfully opened
if not vs.isOpened():
    print("Cannot open camera")
    exit()

while True:
	ret, frame = vs.read()
	frame = imutils.resize(frame, width=1000) # maximum width of 1000 pixels

	# output lists
	a_objects = []
	a_plane = []
	a_positions = []
	a_robot = None
	
	(corners, ids, rejected) = detector.detectMarkers(frame)
	if len(corners) > 0:
		ids = ids.flatten()

		# for each aruco detected
		for (markerCorner, markerID) in zip(corners, ids):
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

			# compute and draw the center (x, y)-coordinates
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

			# draw the ArUco marker ID on the frame
			cv2.putText(frame, str(markerID),
				(topLeft[0], topLeft[1] - 15),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
			
			# formats output lists
			if markerID in ARUCO_OBJECTS:
				# calculate distortion from 3D perspective
				(frame_h, frame_w) = frame.shape[:2]
				distance_from_center = cX - frame_w/2 # stream width center
				distortion = int(DISTORTION_COEFFICIENT * distance_from_center)
				a_objects.append((cX - distortion, cY))
			elif markerID in ARUCO_PLANE:
				a_plane.append((cX, cY))
			if markerID in ARUCO_POSITIONS:
				a_positions.append((cX, cY))
			if markerID == ARUCO_ROBOT:
				a_robot = (cX, cY)

	# output to be replaced by ROS messages
	print("objects\t\t", a_objects)
	print("plane\t\t",a_plane)
	print("positions\t",a_positions)
	print("robot\t\t",a_robot)
	print('-----')
	
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"): # quit
		print("Exiting...")
		break
	
cv2.destroyAllWindows()
vs.release()