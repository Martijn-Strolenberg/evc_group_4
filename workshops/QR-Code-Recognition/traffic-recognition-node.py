import cv2
import time
from pyzbar import pyzbar

def detect_qr_codes(frame):
    # Decode QR codes in the frame
    decoded_objects = pyzbar.decode(frame)
    for obj in decoded_objects:
        # Draw bounding box
        points = obj.polygon
        if len(points) > 4:  # convex hull if needed
            hull = cv2.convexHull(
                np.array([[p.x, p.y] for p in points], dtype=np.float32)
            )
            points = [tuple(point[0]) for point in hull]

        for i in range(len(points)):
            pt1 = (int(points[i][0]), int(points[i][1]))
            pt2 = (int(points[(i + 1) % len(points)][0]), int(points[(i + 1) % len(points)][1]))
            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

        # Print and show the data
        qr_data = obj.data
        qr_type = obj.type
        print("Detected {}: {}".format(qr_type, qr_data))
        cv2.putText(frame, str(qr_data), (points[0][0], points[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    return frame

def main():
    print("Loading image...")

    # Load the PNG image
    frame = cv2.imread("100-distorted-2.png", cv2.IMREAD_UNCHANGED)

    if frame is None:
        print("Error: Could not load image!")
        return

    frame = detect_qr_codes(frame)

if __name__ == '__main__':
    import numpy as np
    main()
