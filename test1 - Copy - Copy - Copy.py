import cv2
import numpy as np

batasLuasAtasMerah = 1000000 
batasLuasBawahMerah = 300 
# Read the image
image = cv2.imread('D:\Resources/a1.jpg')
def rescale(frame, scale=0.75):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)
    return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA)
image = rescale(image,0.5)

# Convert BGR to HSV
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define range of colors in HSV
lower_range = np.array([73, 0, 163])
upper_range = np.array([105, 255, 255])

# Threshold the HSV image to get only desired colors
mask = cv2.inRange(hsv, lower_range, upper_range)

# Find contours
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# contours = max(contours, key=cv2.contourArea)
contoursList = []
for i in contours:
    area1 = cv2.contourArea(i)

    if area1 < batasLuasAtasMerah and area1 > batasLuasBawahMerah:
        contoursList.append(i)
        
contours = tuple(contoursList)

if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        
        area1 = cv2.contourArea(c)
        
        M = cv2.moments(c)
        if M['m00']!=0:
            cxM = int(M['m10']/M['m00'])
            cyM = int(M['m01']/M['m00'])
            cv2.rectangle(image, (cxM-250,cyM-230), (cxM+240,cyM+230), (0,255,0), 1)

# Loop over contours and detect shapes
for contour in contours:
    # Approximate the contour
    approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
    
    # Get the number of vertices
    vertices = len(approx)
    
    # Determine the shape
    if vertices == 3:
        shape_name = "Triangle"
    elif vertices == 4:
        x, y, w, h = cv2.boundingRect(approx)
        aspect_ratio = float(w) / h
        shape_name = "Square" if 0.95 <= aspect_ratio <= 1.05 else "Rectangle"
    elif vertices == 5:
        shape_name = "Pentagon"
    elif vertices == 6:
        shape_name = "Hexagon"
    else:
        shape_name = "Circle"

    # Draw the shape name
    # cv2.drawContours(image, [contour], 0, (0, 255, 0), -1)
    # cv2.putText(image, shape_name, (approx.ravel()[0], approx.ravel()[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.putText(image, shape_name, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

# Display the image
cv2.imshow('Shapes Detected', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
