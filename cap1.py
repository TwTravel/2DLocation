import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(3, 1600);
cap.set(4, 1200);
while(1):
    # get a frame
    ret, frame = cap.read()
    # show a frame
    #cv2.imwrite("capimg.jpg", frame)
    cv2.imshow("capture", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows() 