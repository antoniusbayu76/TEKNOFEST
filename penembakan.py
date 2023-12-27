import cv2
import numpy as np

vid = cv2.VideoCapture(0)
prevCir = None
dist = lambda x1,y1,x2,y2: (x1-x2)**2*(y1-y2)**2

while True:
    ret, frame = vid.read()
    if not ret: break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(21,21),0)

    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1.2, 200,
                               param1=110, param2=50, minRadius=75, maxRadius=400)
    
    
    
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
        print("x =", chosen[0])
        print("y =", chosen[1])
    cv2.imshow("frame",frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows

        