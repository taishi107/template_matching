#!/usr/bin/env python

import cv2

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        print ret
        if not ret:
            continue
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        cv2.imshow("window", grey)
        if cv2.waitKey(1) > 0:
            break
