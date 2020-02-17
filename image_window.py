import cv2

def main():
    cv2.namedWindow("Create View", flags=cv2.WINDOW_AUTOSIZE)
    
    stream = cv2.VideoCapture("http://raspberrypi-1.local:8081")
    
    while True:
        img = []
        stream.read(img)
        cv2.imshow("Create View",img)
        cv2.waitKey(5)
