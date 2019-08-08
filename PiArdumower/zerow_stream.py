import numpy as np
import cv2

cap = cv2.VideoCapture('http://192.168.1.14:8080')

#while(True):

ret, frame = cap.read()
cv2.imshow('Stream IP camera opencv',frame)

#    if cv2.waitKey(1) & 0xFF == ord('q'):
#        break

#cap.release()
#cv2.destroyAllWindows()
