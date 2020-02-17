import cv2

def main():
    cv2.namedWindow("Create View", flags=cv2.WINDOW_AUTOSIZE)
    
    stream = cv2.VideoCapture("http://raspberrypi-2.local:8081")

    while True:
        #img = []
        #stream.read(img)
        ret, frame = stream.read()
        cv2.imshow("Create View",frame)
        cv2.waitKey(5)

main()
