import cv2
import time

# Create a VideoCapture object to read from the RTSP stream
stream_url = "rtsp://host.docker.internal:8554/mystream"

# Open the video stream
cap = cv2.VideoCapture(stream_url)
#cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Set buffer size to 1 frame

if not cap.isOpened():
    print("ERROR: Failed to open video stream.")
    exit(1)

print("Stream opened successfully. Press 'q' to quit.")

while True:
    # Flush frames
    for _ in range(5):
        cap.grab()  # grab but don't decode

    ret, frame = cap.read()
    if not ret:
        print("ERROR: Failed to grab frame.")
        break

    cv2.imshow("RTSP Stream", frame)
    
    print("Frame time: %f" % time.time())

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()