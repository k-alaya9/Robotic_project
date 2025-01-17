import cv2 as cv
import numpy as np

# Capture the image from the Webots camera
raw_image = camera.getImage()
image = np.frombuffer(raw_image, dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
bgr_image = cv.cvtColor(image, cv.COLOR_BGRA2BGR)

# Define color ranges (in HSV) for detection
color_ranges = {
    "red": ((0, 100, 100), (10, 255, 255)),  # Lower and upper bounds for red
    "green": ((40, 50, 50), (80, 255, 255)), # Lower and upper bounds for green
    "blue": ((100, 150, 0), (140, 255, 255)),# Lower and upper bounds for blue
    "yellow": ((20, 100, 100), (30, 255, 255)) # Lower and upper bounds for yellow
}

# Convert to HSV
hsv_image = cv.cvtColor(bgr_image, cv.COLOR_BGR2HSV)

detected_positions = {}

# Detect boxes by color
for color, (lower, upper) in color_ranges.items():
    mask = cv.inRange(hsv_image, np.array(lower), np.array(upper))
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        # Calculate bounding box
        x, y, w, h = cv.boundingRect(contour)
        cv.rectangle(bgr_image, (x, y), (x + w, y + h), (255, 255, 255), 2)
        
        # Save position as the center of the box
        center_x = x + w // 2
        center_y = y + h // 2
        detected_positions[color] = (center_x, center_y)
        
        # Annotate the box
        cv.putText(bgr_image, color, (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

# Show the detected boxes
cv.imshow("Detected Boxes", bgr_image)
cv.waitKey(1)

print("Detected positions:", detected_positions)
