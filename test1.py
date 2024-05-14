import cv2
import numpy as np

# Baca gambar dari file
gambar = cv2.imread('D:\Resources/t1.jpg')
prevCir = None
shoot = 15
scalling = 1
dist = lambda x1,y1,x2,y2: (x1-x2)**2*(y1-y2)**2
lower = np.array([17, 0, 0])
upper = np.array([179, 255, 255])
k = 1
def rescale(frame, scale=0.75):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)
    return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA)
frame = rescale(gambar,0.5)
# Tampilkan gambar

# Tunggu tombol keyboard ditekan dan tutup jendela ketika tombol q ditekan
while True:
    

    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(frame,(21,21),0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1.2, 500,
                               param1=100, param2=20, minRadius=75, maxRadius=240)
    
    cv2.line(frame, (0,frame.shape[0]//2), (frame.shape[1],frame.shape[0]//2), (0,0,0), thickness=2)
    cv2.line(frame, (frame.shape[1]//2,0), (frame.shape[1]//2,frame.shape[0]), (0,0,0), thickness=2)
    cv2.circle(frame, (frame.shape[1]//2,frame.shape[0]//2),shoot,(0,0,255),3)
    
    if circles is not None:
        if k == 1 :
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
            k = 0

        if chosen[0] > frame.shape[1]//2 - shoot and chosen[0] < frame.shape[1]//2 + shoot and chosen[1] > frame.shape[0]//2 - shoot and chosen[1] < frame.shape[0]//2 + shoot:
            cv2.putText(frame, "Shoot", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        if chosen[0] < frame.shape[1]//2 - shoot :
            cv2.putText(frame, "Go Left", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        elif chosen[0] > frame.shape[1]//2 + shoot :
            cv2.putText(frame, "Go Right", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        if chosen[1] < frame.shape[0]//2 - shoot :
            cv2.putText(frame, "Go Up", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        elif chosen[1] > frame.shape[0]//2 + shoot :
            cv2.putText(frame, "Go Down", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # print("x =", chosen[0])
        # print("y =", chosen[1])
    cv2.imshow("frame",frame)
    cv2.imshow("mask",mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Tutup jendela
cv2.destroyAllWindows()
