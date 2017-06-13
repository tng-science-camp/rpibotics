import cv2
import numpy
import picamera.array

BLUE_HSV_LOWER_RANGE = (96, 73, 0)
BLUE_HSV_UPPER_RANGE = (128, 255, 255)

def find_colored_object(stream: picamera.array.PiArrayOutput,
                        hsv_lower_range, hsv_upper_range):
    image = stream.array
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower_range, hsv_upper_range)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
    x = None
    y = None
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
    return x, y

def find_colored_object_bearing(stream: picamera.array.PiArrayOutput,
                                hsv_lower_range, hsv_upper_range):
    x, y = find_colored_object(stream, hsv_lower_range, hsv_upper_range)
    # Angle of View: 62.2 x 48.8 degrees
    if x != None and y != None:
        width, height = stream.array.shape
        return numpy.arctan((x - width / 2) / (width/ 2 / numpy.tan(numpy.deg2rad(31.1))))
    else:
        return None