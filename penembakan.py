import cv2
import numpy as np
import pidControl as ct2
import controlTest as ct1
import servo as sv
import time

# Inisialisasi video capture
vid = cv2.VideoCapture(0)
prevCir = None
shoot = 30
scaling = 1

# Fungsi untuk menghitung jarak antara dua titik
dist = lambda x1, y1, x2, y2: (x1 - x2) ** 2 + (y1 - y2) ** 2

# Parameter luas target
setArea = 0  # luas target pas nembak
minArea = 0  # luas max target yang bisa dibaca
maxArea = 0  # luas min target yang bisa dibaca

# Waktu untuk gerakan awal
detikMuter = 3
detikMaju = 5
detikNyelem = 4

nyelam = 200

# Fungsi untuk mengubah ukuran frame
def rescale(frame, scale=0.75):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)
    return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA)

# Rentang warna untuk deteksi objek
lower = np.array([100, 0, 237])
upper = np.array([113, 255, 255])

# Mencoba untuk mengarm kendaraan
try:
    ct2.arm_vehicle()
except:    
    print("Ga Bisa Arming")

# Gerakan awal
for _ in range(detikNyelem):
    ct1.send_manual_control(0, 0, nyelam, 0)
    print("Nyelem dulu")
    time.sleep(1)
for _ in range(detikMuter):
    ct1.send_manual_control(0, 0, nyelam, 700)
    print("Muter")
    time.sleep(1)
for _ in range(detikMaju):
    ct1.send_manual_control(-500, 0, nyelam, 0)
    print("Maju")
    time.sleep(1)

# Proses utama
while True:
    ret, frame = vid.read()
    if not ret: break

    frame = rescale(frame, scaling)

    # Proses deteksi objek
    blur = cv2.GaussianBlur(frame, (21, 21), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1.2, 500,
                               param1=100, param2=20, minRadius=75, maxRadius=300)
    
    # Garis dan lingkaran panduan
    cv2.line(frame, (0, frame.shape[0] // 2), (frame.shape[1], frame.shape[0] // 2), (0, 0, 0), thickness=2)
    cv2.line(frame, (frame.shape[1] // 2, 0), (frame.shape[1] // 2, frame.shape[0]), (0, 0, 0), thickness=2)
    cv2.circle(frame, (frame.shape[1] // 2, frame.shape[0] // 2), shoot, (0, 0, 255), 3)
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        chosen = None
        for i in circles[0, :]:
            if chosen is None:
                chosen = i
            if prevCir is not None:
                if dist(chosen[0], chosen[1], prevCir[0], prevCir[1]) <= dist(i[0], i[1], prevCir[0], prevCir[1]):
                    chosen = i
        cv2.circle(frame, (chosen[0], chosen[1]), 1, (0, 255, 0), 3)
        cv2.circle(frame, (chosen[0], chosen[1]), chosen[2], (0, 0, 255), 3)
        prevCir = chosen

        xAxis = 0
        yAxis = chosen[0]
        zAxis = chosen[1]

        # Kontrol gerakan berdasarkan posisi lingkaran yang terdeteksi
        if chosen[2] > 100 and chosen[2]< 120:
            if chosen[0] > frame.shape[1] // 2 - shoot and chosen[0] < frame.shape[1] // 2 + shoot and chosen[1] > frame.shape[0] // 2 - shoot and chosen[1] < frame.shape[0] // 2 + shoot:
                sv.move_continuous_servo(11, 1900)
                print("Tembakkkkkkk")
            else:
                sv.move_continuous_servo(11, 1500)
            if chosen[0] < frame.shape[1] // 2 - shoot:
                ct1.send_manual_control(0, 500, nyelam, 0)
                print("Kiri")
            elif chosen[0] > frame.shape[1] // 2 + shoot:
                ct1.send_manual_control(0, -500, nyelam, 0)
                print("Kanan")
            if chosen[1] < frame.shape[0] // 2 - shoot:
                ct1.send_manual_control(0, 0, nyelam + 100, 0)
                print("Naik")
            elif chosen[1] > frame.shape[0] // 2 + shoot:
                ct1.send_manual_control(0, 0, nyelam - 100, 0)
                print("Turun")
        elif chosen[2] < 100:
            ct1.send_manual_control(-500, 0, nyelam, 0)
            print("maju")
        else :
            ct1.send_manual_control(500, 0, nyelam, 0)
            print("Mundur")


    cv2.imshow("frame", frame)
    # cv2.imshow("mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()
