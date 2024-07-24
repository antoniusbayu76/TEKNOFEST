import cv2
import numpy as np
import controlTest as ct1
# import servo as sv
import time

# Base speed values
nyelam = 230
penetral = -100

# # Initialize video capture
# path = "/home/gamantaray/Documents/TEKNOFEST/kalibrasi.avi"
# vid = cv2.VideoCapture(0)
# prevCir = None
# shoot = 40
# scaling = 1

# # Function to calculate distance between two points
# def dist(x1, y1, x2, y2):
#     return (x1 - x2) ** 2 + (y1 - y2) ** 2

# Timing for initial movement
detikMaju = 4
detikNyelem = 7

# # Function to resize the frame
# def rescale(frame, scale=0.75):
#     width = int(frame.shape[1] * scale)
#     height = int(frame.shape[0] * scale)
#     dimensions = (width, height)
#     return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA)

# # Color range for object detection
# lower = np.array([93, 250, 194])
# upper = np.array([103, 255, 255])

# min_area = 100000
# max_area = 120000

# Attempt to arm the vehicle
try:
    ct1.arm_vehicle()
except:
    print("Cannot arm the vehicle")

# Initial movement with while loop
start_time = time.time()
while True:
    current_time = time.time()
    elapsed_time = current_time - start_time

    if elapsed_time < detikNyelem:
        ct1.send_manual_control(0, 0, (nyelam - 150), 0)
        print("Diving")
    elif elapsed_time < (detikNyelem+detikMaju):
        ct1.send_manual_control(-200,0,nyelam,0)
    else:
        ct1.send_manual_control(0, 0, 500, 0)
        break

# Main movement loop (currently commented out)
# Uncomment and adjust as needed

# # Initialize VideoWriter
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('output3.avi', fourcc, 20.0, (int(vid.get(3)*scaling), int(vid.get(4)*scaling)))
# out2 = cv2.VideoWriter('output4.avi', fourcc, 20.0, (int(vid.get(cv2.CAP_PROP_FRAME_WIDTH) * scaling), int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT) * scaling)))

# # Main movement
# while True:
#     ret, frame = vid.read()
#     if not ret:
#         break

#     frame = rescale(frame, scaling)

#     # Object detection processing
#     blur = cv2.GaussianBlur(frame, (21, 21), 0)
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#     mask = cv2.inRange(hsv, lower, upper)
#     mask = rescale(mask, scaling)

#     circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1.2, 500,
#                                param1=100, param2=15, minRadius=75, maxRadius=300)

#     # Guide lines and circles
#     cv2.line(frame, (0, frame.shape[0] // 2), (frame.shape[1], frame.shape[0] // 2), (0, 0, 0), thickness=2)
#     cv2.line(frame, (frame.shape[1] // 2, 0), (frame.shape[1] // 2, frame.shape[0]), (0, 0, 0), thickness=2)
#     cv2.circle(frame, (frame.shape[1] // 2, frame.shape[0] // 2), shoot, (0, 255, 0), 3)

#     if circles is not None:
#         circles = np.uint16(np.around(circles))
#         chosen = None
#         for i in circles[0, :]:
#             area = np.pi * (i[2] ** 2)
#             if min_area <= area <= max_area:
#                 if chosen is None:
#                     chosen = i
#                 if prevCir is not None:
#                     if dist(chosen[0], chosen[1], prevCir[0], prevCir[1]) <= dist(i[0], i[1], prevCir[0], prevCir[1]):
#                         chosen = i

#         if chosen is not None:
#             cv2.circle(frame, (chosen[0], chosen[1]), 1, (0, 255, 0), 3)
#             cv2.circle(frame, (chosen[0], chosen[1]), chosen[2], (0, 255, 0), 3)
#             area_text = f'Area: {int(np.pi * (chosen[2] ** 2))}'
#             cv2.putText(frame, area_text, (0, frame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
#             prevCir = chosen

#             if chosen[0] < frame.shape[1] // 2 - shoot:
#                 ct1.send_manual_control(0, penetral + 50, nyelam, 0)
#                 print("Left")
#                 cv2.putText(frame, "Left", (frame.shape[1] - 200, frame.shape[0] - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
#             elif chosen[0] > frame.shape[1] // 2 + shoot:
#                 ct1.send_manual_control(0, penetral - 50, nyelam, 0)
#                 print("Right")
#                 cv2.putText(frame, "Right", (frame.shape[1] - 200, frame.shape[0] - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
#             if chosen[1] < frame.shape[0] // 2 - shoot:
#                 ct1.send_manual_control(0, penetral, nyelam + 50, 0)
#                 print("Up")
#                 cv2.putText(frame, "Up", (frame.shape[1] - 200, frame.shape[0] - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
#             elif chosen[1] > frame.shape[0] // 2 + shoot:
#                 ct1.send_manual_control(0, penetral, nyelam - 50, 0)
#                 print("Down")
#                 cv2.putText(frame, "Down", (frame.shape[1] - 200, frame.shape[0] - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
#             else:
#                 ct1.send_manual_control(0, penetral, nyelam , 0)

#             if np.pi * (chosen[2] ** 2) > 105000:
#                 sv.move_continuous_servo(11, 1900)
#                 print("servo on")
#                 cv2.putText(frame, "servo on", (frame.shape[1] - 200, frame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
#             else:
#                 sv.move_continuous_servo(11, 1500)
#                 ct1.send_manual_control(-100, penetral, nyelam, 0)
#                 print("servo off")
#                 cv2.putText(frame, "servo off", (frame.shape[1] - 200, frame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

#         else:
#             sv.move_continuous_servo(11, 1500)    
#             print("servo off")
#             cv2.putText(frame, "servo off", (frame.shape[1] - 200, frame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

#     # Write the frame to the video file
#     out.write(frame)
#     out2.write(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR))

#     cv2.imshow("frame", frame)
#     # cv2.imshow("mask", mask)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # Release everything when done
# sv.move_continuous_servo(11, 1500)
# vid.release()
# out.release()
# out2.release()
# cv2.destroyAllWindows()
