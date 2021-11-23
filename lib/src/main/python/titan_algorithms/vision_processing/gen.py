# pip install opencv-contrib-python


import cv2
import os
import numpy
from math import *
cv = cv2
np = numpy


class Line:
    def __init__(self, *args):
        if len(args) == 2:
            self.x1 = args[0][0]
            self.y1 = args[0][1]
            self.x2 = args[1][0]
            self.y2 = args[1][1]
        elif len(args) == 4:
            self.x1, self.y1, self.x2, self.y2 = args
        else:
            raise TypeError("Line takes either 2 or 4 arguments, representing the endpoints of the line")

    def __len__(self):
        return int(self.length)

    @property
    def length(self):
        return sqrt(pow(abs(self.x1 - self.x2), 2) + pow(abs(self.y1 - self.y2), 2))


try:
    os.chdir(os.path.dirname(__file__))  # Set current working directory to script folder
except NameError:
    print("Running in " + os.getcwd())
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# https://www.pyimagesearch.com/2020/12/14/generating-aruco-markers-with-opencv-and-python/
def draw(id):
    marker = cv2.aruco.drawMarker(arucoDict, id, 300)
    cv2.imshow('test', marker)
    return marker

# https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
def track(image):
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    if len(corners) > 0:
        for shape, id in zip(corners[0], ids):  # TODO: rename shape to boundingSqr or something
            shape = shape.astype(numpy.int32)
            print(shape, id)
            # IMPORTANT: this assumes that the marker is oriented upright
            width = Line(shape[0], shape[1]).length
            height = Line(shape[1], shape[2]).length
            print(width, 'x', height)
            # TODO: this doesn't work at all, can't determine which direction or angle
            print("predicted angle:", atan((height - width) / height) * 180 / pi)
            cv2.drawContours(image, [shape], 0, (0, 0, 255), 2)
            cv.line(image, shape[0], shape[1], (255,0,0), 5)

def mainloop():
    while True:
        # Hit 'q' on the keyboard to quit!
        key = cv2.waitKeyEx(30) & 0xFF
        frame = cap.read()[1]
        track(frame)

        ''' For testing purposes
        imgray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        ret, thresh = cv.threshold(imgray, 127, 255, 0)
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, (0, 255, 0))'''

        cv2.imshow('webcam', frame)
        if key == ord('q'):
            break

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    marker = draw(0)
    # cv2.imwrite("0.png", marker)
    track(marker)
    try:
        mainloop()
    finally:
        cv2.destroyAllWindows()
        cap.release()
