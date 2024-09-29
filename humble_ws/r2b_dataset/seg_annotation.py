import cv2
import numpy as np
import os

# Global variables
points = []

# Mouse callback function
def mouse_callback(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        cv2.circle(image, (x, y), 1, (0, 0, 255), -1)
        if len(points) > 1:
            cv2.line(image, points[-2], points[-1], (0, 0, 255), 1)
        cv2.imshow("image", image)
    elif event == cv2.EVENT_RBUTTONDOWN:
        if len(points) > 2:
            fill_polygon()

# Function to fill the polygon
def fill_polygon():
    global points, i, mask
    
    pts = np.array(points, np.int32)
    pts = pts.reshape((-1, 1, 2))
    cv2.fillPoly(mask, [pts], (255, 255, 255))
    points = []
    #result = cv2.bitwise_and(image, mask)
    cv2.imshow("Filled Polygon", mask)
    cv2.imwrite(f'r2b_hallway_seglabel/{i:06d}.png', mask)


bev_file_list = os.listdir('/home/danilo/r2b_dataset/r2b_hallway_bev')
bev_files = sorted([file for file in bev_file_list if file.lower().endswith('.png')])
i=39

for bev_file in bev_files[:]:
    # Load the image
    image = cv2.imread(f'r2b_hallway_bev/{i:06d}.png')
    mask = np.zeros_like(image)

    # Create a window and set the mouse callback function
    cv2.namedWindow("image")
    cv2.setMouseCallback("image", mouse_callback)

    # Display the image
    cv2.imshow("image", image)

    # Wait for key press and close OpenCV windows
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    break
    i+=1
