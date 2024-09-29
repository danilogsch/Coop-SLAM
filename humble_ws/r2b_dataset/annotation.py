import cv2
import math
import numpy as np
import os

# List to store mouse click positions
positions = []
# Function to calculate distance between two points
def calculate_distance(point1, point2):
    dy = point2[0] - point1[0]
    dx = point2[1] - point1[1]
    distance = math.sqrt(dx ** 2 + dy ** 2)
    return distance

# Function to calculate angle between two points
def calculate_angle(point1, point2):
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    angle = math.atan2(dy, dx)
    return angle

# Function to calculate middle point between two points
def calculate_middle_point(point1, point2):
    y_mid = (point1[0] + point2[0]) / 2
    x_mid = (point1[1] + point2[1]) / 2
    return (x_mid, y_mid)

# Function to handle mouse clicks
def mouse_callback(event, x, y, flags, param):
    global positions, first_click, RES, x_m, y_m, w, l, theta, i
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Mouse clicked at position (x={}, y={})".format(x, y))
        positions.append((x, y))
        cv2.circle(image, (x, y), 2, (0, 0, 255), -1)
        if len(positions) == 2 and first_click == 0:
            cv2.line(image, positions[-2], positions[-1], (0, 0, 255), 1)
            first_click = 1
            w = round(calculate_distance(positions[0], positions[1]))
            theta = calculate_angle(positions[0], positions[1])
            theta = (math.pi/2) - theta
            x_m, y_m = calculate_middle_point(positions[0], positions[1])
            x_m = x_m * RES
            y_m = (y_m-304) * RES
            print("Distance between the points (in pixels): {:.2f}".format(w))
            print("Angle (in radians) of the line: {:.2f}".format(theta))
            print("Middle point of the line: ({:.2f}, {:.2f})".format(x, y))
            positions = []
        if len(positions) == 2 and first_click == 1:
            cv2.line(image, positions[-2], positions[-1], (0, 0, 255), 1)
            first_click = 0
            l = round(calculate_distance(positions[0], positions[1]))
            print("Distance between the points (in pixels): {:.2f}".format(l))
            positions = []
            
            # Save the pixel positions to a text file
            with open(f'r2b_hallway_label/{i:06d}.txt', 'a') as file:
                file.write('0 {} {} 0 {} {} 1.8 {}\n'.format(x_m, y_m, w, l, theta))
        cv2.imshow("image", image)

first_click = 0
RES = 10 / 608
print(RES)
x_m = 0
y_m = 0
w = 0
l = 0
theta = 0

i = 0

bev_file_list = os.listdir('/home/danilo/r2b_dataset/r2b_hallway_bev')
bev_files = sorted([file for file in bev_file_list if file.lower().endswith('.png')])

for bev_file in bev_files[:]:

    # Load the image
    image = cv2.imread(f'r2b_hallway_bev/{i:06d}.png')

    # Create a window and set the mouse callback function
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', mouse_callback)

    

    # Display the image and wait for user input
    cv2.imshow('image', image)
    print("Click on the image to annotate the object:")
    print("1. Click to select two points to calculate distance, angle, and middle point.")
    print("2. Click again to select two more points to calculate the distance between them.")
    print("Press 'q' to exit.")
    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    i += 1
    break
    # Close OpenCV windows
    cv2.destroyAllWindows()
