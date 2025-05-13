import sys
import numpy as np
import cv2

width, height = 640, 480

while True:
    raw = sys.stdin.read(width * height * 3)

    print("Read bytes:", len(raw))
    if len(raw) < width*height*3:
        print("** Short read! **")
        
    if not raw:
        break
    frame = np.fromstring(raw, dtype=np.uint8).reshape((height, width, 3))
    cv2.imshow('Live Stream', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break