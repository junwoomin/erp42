#! /usr/bin/env python3
import cv2
import numpy as np
from keras.models import load_model
import rospy
from std_msgs.msg import Int32
rospy.init_node('cv2.steer',anonymous=True)
steer_pub=rospy.Publisher("/c_steer",Int32,queue_size=3)

def preprocess_frame(frame, img_height, img_width):
    frame_resized = cv2.resize(frame, (img_width, img_height))
    frame_normalized = frame_resized / 255.0
    return np.expand_dims(frame_normalized, axis=0)
def find_intersection(line1, line2):
    if line1[0] == line2[0]:
        return None  # Parallel lines
    x = (line2[1] - line1[1]) / (line1[0] - line2[0])
    y = line1[0] * x + line1[1]
    return round(x, 4), round(y, 4)
def get_line_eq(x1, y1, x2, y2):
    slope = (y2 - y1) / (x2 - x1)
    intercept = y1 - slope * x1
    return slope, intercept
def filter_lines_by_angle(lines, min_angle, max_angle):
    filtered_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
        if min_angle <= angle <= max_angle or (-max_angle <= angle <= -min_angle):
            filtered_lines.append(line)
    return filtered_lines

model = load_model("catkin_ws/src/erp_driver/scripts/lane.h5")

cap = cv2.VideoCapture(4)
img_height, img_width = 256, 256
while True:
    ret, frame = cap.read()
    frame_height, frame_width, _ = frame.shape
    if not ret:
        break
    input_frame = preprocess_frame(frame, img_height, img_width)

    prediction = model.predict(input_frame)
    mask = cv2.resize(prediction.squeeze(), (frame.shape[1], frame.shape[0]))
    mask = (mask * 255).astype(np.uint8)
    edges = cv2.Canny(mask, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)
    min_angle = 0
    max_angle = 120
    if lines is not None:
        filtered_lines = filter_lines_by_angle(lines, min_angle, max_angle)
    else:
        filtered_lines = []

    if filtered_lines:
        po=0
        m=0
        x1, y1, x2, y2 = filtered_lines[0][0]
        li =0
        line_a = get_line_eq(x1, y1, x2, y2)
        filtered_lines = np.delete(filtered_lines, 0, axis=0)
        point = 0
        for line in filtered_lines:
            x1, y1, x2, y2 = line[0]
            line_b = get_line_eq(x1, y1, x2, y2)
            intersection_point = find_intersection(line_a, line_b)
            if intersection_point is not None and 0<intersection_point[0]<=frame_width:
                li=li+1
                point = intersection_point[0] + point
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        if li > 0:
            point_sum = point / li
            st=point_sum/frame_width
            steer=0        
            if 0.4>st:
                steer=-st*2000
            if 0.6<st:
                steer=st*-2000
            steer_pub.publish(int(steer))
            print(steer)

    a=cv2.resize(mask, (640,480))
    b=cv2.resize(frame, (640,480))
    cv2.imshow("mask", a)
    cv2.imshow("frame", b)
    

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
