import cv2
import numpy as np

# Baca gambar dari file
gambar = cv2.imread('D:\Resources/t2.jpg')
prevCir = None
shoot = 15
scalling = 1
dist = lambda x1,y1,x2,y2: (x1-x2)**2*(y1-y2)**2
lower = np.array([13, 0, 0])
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
    contours2,hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)



    c = max(contours2, key=cv2.contourArea)
        
    area2 = cv2.contourArea(c)
    if k == 1:
        M = cv2.moments(c)
        if M['m00']!=0:
            cxH = int(M['m10']/M['m00'])
            cyH = int(M['m01']/M['m00'])
            cv2.rectangle(frame, (cxH-70,cyH-130), (cxH+70,cyH+120), (0,255,0), 1)
            cv2.circle(frame, (cxH,cyH), 5, (0,255,0), thickness=2)
            cv2.putText(frame, "RAL Codes = 6018", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        k = 0
    # print("x =", chosen[0])
        # print("y =", chosen[1])
    
    cv2.imshow("frame",frame)
    cv2.imshow("mask",mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Tutup jendela
cv2.destroyAllWindows()
