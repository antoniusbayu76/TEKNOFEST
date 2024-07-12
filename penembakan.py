import cv2
import numpy as np
import pidControl as ct

vid = cv2.VideoCapture(0)
prevCir = None
shoot = 15
scalling = 1
dist = lambda x1,y1,x2,y2: (x1-x2)**2*(y1-y2)**2

setArea = 0 #luas target pas nembak
minArea = 0 #luas max target yang bisa dibaca
maxArea = 0 #luas min target yang bisa dibaca

def rescale(frame, scale=0.75):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)
    return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA)

lower = np.array([100, 0, 237])
upper = np.array([113, 255, 255])

try:
    ct.arm_vehicle()
except:    
    print("Ga Bisa Arming")

while True:
    ret, frame = vid.read()
    if not ret: break

    frame = rescale(frame,scalling)

    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(frame,(21,21),0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1.2, 500,
                               param1=100, param2=20, minRadius=75, maxRadius=300)
    
    cv2.line(frame, (0,frame.shape[0]//2), (frame.shape[1],frame.shape[0]//2), (0,0,0), thickness=2)
    cv2.line(frame, (frame.shape[1]//2,0), (frame.shape[1]//2,frame.shape[0]), (0,0,0), thickness=2)
    cv2.circle(frame, (frame.shape[1]//2,frame.shape[0]//2),shoot,(0,0,255),3)
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        chosen = None
        for i in circles[0,:]:
            if chosen is None:
                chosen = i
            if prevCir is not None:
                if dist(chosen[0],chosen[1],prevCir[0],prevCir[1]) <= dist(i[0],i[1],prevCir[0],prevCir[1]):
                    chosen = i
        cv2.circle(frame, (chosen[0],chosen[1]),1,(0,255,0),3)
        cv2.circle(frame, (chosen[0],chosen[1]),chosen[2],(0,0,255),3)
        prevCir = chosen

        xAxis = 0
        yAxis = chosen[0]
        zAxis = chosen[1]

        ct.send_manual_control(xAxis,yAxis,zAxis,setArea,frame.shape[1]//2,frame.shape[0]//2,minArea,maxArea,0,frame.shape[1],0,frame.shape[0])


        # if chosen[0] > frame.shape[1]//2 - shoot and chosen[0] < frame.shape[1]//2 + shoot and chosen[1] > frame.shape[0]//2 - shoot and chosen[1] < frame.shape[0]//2 + shoot:
        #     print("tembakkkkkkk")
        # if chosen[0] < frame.shape[1]//2 - shoot :
        #     print("kaaaaaaaaaaaaaaaaaanan")
        # elif chosen[0] > frame.shape[1]//2 + shoot :
        #     print("kiiiiiiiiiiiiiiiiiiri")
        # if chosen[1] < frame.shape[0]//2 - shoot :
        #     print("tuuuuuuuuuuuuuuuuurun")
        # elif chosen[1] > frame.shape[0]//2 + shoot :
        #     print("naiiiiiiiiiiiiiiiiiiik")
        # # print("x =", chosen[0])
        # # print("y =", chosen[1])



    cv2.imshow("frame",frame)
    cv2.imshow("mask",mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows

        