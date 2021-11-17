# pip install opencv-contrib-python


import cv2

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
    print(corners, ids, rejected)

def mainloop():
    while True:
        # Hit 'q' on the keyboard to quit!
        key = cv2.waitKeyEx(30) & 0xFF
        cv2.imshow('webcam', cap.read()[0])
        if key == ord('q'):
            break

if __name__ == "__main__":
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    track(draw(0))
    mainloop()
