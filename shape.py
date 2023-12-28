import cv2
import numpy as np

vid = cv2.VideoCapture(0)

def empty(a):
    pass

def getShape(img,imgContour):
    contours,hierarchy =  cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contoursList = []
    for i in contours:
        area = cv2.contourArea(i)

        if area > 1000:
            contoursList.append(i)
    contours = tuple(contoursList)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
    
        area = cv2.contourArea(c)
        cv2.drawContours(imgContour, c, -1, (255,255,255),7)
        peri = cv2.arcLength(c,True)
        approx = cv2.approxPolyDP(c,0.02*peri,True)
        print(len(approx))
            

cv2.namedWindow("l")
cv2.resizeWindow("l",640,240)
cv2.createTrackbar("t1","l",150,255,empty)
cv2.createTrackbar("t2","l",255,255,empty)


while True :
    ret, frame = vid.read()
    if not ret: break
    imgContour = frame.copy()

    blur = cv2.GaussianBlur(frame,(21,21),0)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

    t1 = cv2.getTrackbarPos("t1","l")
    t2 = cv2.getTrackbarPos("t2","l")
    canny = cv2.Canny(gray,t1,t2)

    kernel = np.ones((5,5))
    dil = cv2.dilate(canny,kernel,iterations = 1)

    getShape(dil,imgContour)

    cv2.imshow("can",imgContour)
    cv2.imshow("frame",frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows
