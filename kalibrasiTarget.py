import cv2
import numpy as np

# Inisialisasi video capture
vid = cv2.VideoCapture(0)
prevCir = None
shoot = 15
scaling = 1
dist = lambda x1, y1, x2, y2: (x1 - x2) ** 2 + (y1 - y2) ** 2  # Fungsi lambda untuk menghitung jarak

# Fungsi untuk mengubah ukuran frame
def rescale(frame, scale=0.75):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)
    return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA)

# Rentang warna untuk deteksi objek
lower = np.array([103, 103, 195])
upper = np.array([111, 255, 255])

while True:
    ret, frame = vid.read()
    if not ret:
        break

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

        # Menulis radius lingkaran yang dipilih di layar
        radius_text = f"Radius: {chosen[2]}"
        cv2.putText(frame, radius_text, (chosen[0] - 50, chosen[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        prevCir = chosen

     # Kontrol gerakan berdasarkan posisi lingkaran yang terdeteksi
        if chosen[2] > 90 and chosen[2]< 120:
            if chosen[0] > frame.shape[1] // 2 - shoot and chosen[0] < frame.shape[1] // 2 + shoot and chosen[1] > frame.shape[0] // 2 - shoot and chosen[1] < frame.shape[0] // 2 + shoot:
                print("Tembakkkkkkk")
            if chosen[0] < frame.shape[1] // 2 - shoot:
                print("Kiri")
            elif chosen[0] > frame.shape[1] // 2 + shoot:
                print("Kanan")
            if chosen[1] < frame.shape[0] // 2 - shoot:
                print("Naik")
            elif chosen[1] > frame.shape[0] // 2 + shoot:
                print("Turun")
        elif chosen[2] > 120:
            print("maju")
           
        else :
            print("Mundur")

    cv2.imshow("frame", frame)
    # cv2.imshow("mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()
