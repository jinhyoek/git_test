import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import math

GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)

GPIO_DIR1 = 6
GPIO_PWM1 = 5
GPIO_DIR2 = 20
GPIO_PWM2 = 21

GPIO.setup(GPIO_DIR1, GPIO.OUT)
GPIO.setup(GPIO_PWM1, GPIO.OUT)
GPIO.setup(GPIO_DIR2, GPIO.OUT)
GPIO.setup(GPIO_PWM2, GPIO.OUT)

size_x = 360
size_y = 240


def setspeed(speed1, speed2):
    p1.ChangeDutyCycle(speed1)
    p2.ChangeDutyCycle(speed2)


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

try:
    p1 = GPIO.PWM(GPIO_PWM1, 10)  # 100hz
    p1.start(0)  # start the PWM on 0% duty cycle
    p2 = GPIO.PWM(GPIO_PWM2, 10)  # 100hz
    p2.start(0)  # start the PWM on 0% duty cycle

    while True:
        ret, frame = cap.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        
        GPIO.output(GPIO_DIR1, GPIO.HIGH)
        GPIO.output(GPIO_DIR2, GPIO.HIGH)

        # Set HSV Value
        low_red = np.array([90, 120, 70])
        high_red = np.array([120, 255, 255])
        mask1 = cv2.inRange(hsv_frame, low_red, high_red)

        lower_red = np.array([60, 120, 70])
        upper_red = np.array([90, 255, 255])
        mask2 = cv2.inRange(hsv_frame, lower_red, upper_red)

        mask_red = mask1 + mask2

        red = cv2.bitwise_and(frame, frame, mask=mask_red)

        # binary
        frame_gray = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
        _, img_binary = cv2.threshold(frame_gray, 80, 255, 0)
        contours, hierarchy = cv2.findContours(img_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        maxArea = [0, 0]
        for cnt in contours:
            size = cv2.contourArea(cnt)
            if size > maxArea[0]:
                maxArea[0] = size
                maxArea[1] = cnt

        if maxArea[0] != 0:
            print("area", maxArea[0])
            epsilon = 0.05 * cv2.arcLength(maxArea[1], True)
            approx = cv2.approxPolyDP(maxArea[1], epsilon, True)
            cv2.drawContours(frame, [approx], 0, (0, 0, 255), 5)

            cornerList = [0, 0, 0, 0]
            lengthList = [0, 0, 9999999, 9999999]
            if len(approx) == 4:
                for point in approx:
                    lenFromZero = point[0][0] ** 2 + point[0][1] ** 2
                    if lenFromZero > lengthList[0]:
                        lengthList[0] = lenFromZero
                        cornerList[0] = point[0]
                    if lengthList[3] > lenFromZero:
                        lengthList[3] = lenFromZero
                        cornerList[3] = point[0]

                    lenFromMax = (size_x - point[0][0]) ** 2 + point[0][1] ** 2
                    if lenFromMax > lengthList[1]:
                        lengthList[1] = lenFromMax
                        cornerList[1] = point[0]
                    if lengthList[2] > lenFromMax:
                        lengthList[2] = lenFromMax
                        cornerList[2] = point[0]

                if cornerList[0].any() and cornerList[1].any() and cornerList[2].any() and cornerList[3].any():
                    cv2.circle(frame, tuple(cornerList[0]), 5, (255, 255, 0), 3)
                    cv2.circle(frame, tuple(cornerList[3]), 5, (0, 255, 255), 3)

                    cv2.circle(frame, tuple(cornerList[1]), 5, (255, 0, 0), 3)
                    cv2.circle(frame, tuple(cornerList[2]), 5, (0, 255, 0), 3)

                    cv2.line(frame, tuple(cornerList[0]), tuple(cornerList[2]), (255, 128, 128), 3)
                    cv2.line(frame, tuple(cornerList[3]), tuple(cornerList[1]), (255, 128, 128), 3)

                    lineLeft = math.hypot(cornerList[0][0] - cornerList[2][0], cornerList[0][1] - cornerList[2][1])
                    lineRight = math.hypot(cornerList[1][0] - cornerList[3][0], cornerList[1][1] - cornerList[3][1])
                    print("line", lineLeft, lineRight)

                    if lineLeft > lineRight:
                        cv2.circle(frame, (int(size_x / 2), int(size_y / 2)), 5, (255, 0, 0), 3)
                        GPIO.output(GPIO_DIR1, GPIO.HIGH)
                        GPIO.output(GPIO_DIR2, GPIO.LOW)
                        setspeed(7, 7)
                        time.sleep(3)
                        print("right")
                    else:
                        cv2.circle(frame, (int(size_x / 2), int(size_y / 2)), 5, (0, 255, 0), 3)
                        GPIO.output(GPIO_DIR1, GPIO.LOW)
                        GPIO.output(GPIO_DIR2, GPIO.HIGH)
                        setspeed(7, 7)
                        time.sleep(3)
                        print("left")

                    angleBottom = math.atan2(cornerList[3][0] - cornerList[2][0], cornerList[3][1] - cornerList[2][1])
                    angleBottom = math.degrees(angleBottom)
                    print("angle", angleBottom)

        cv2.imshow("binary", img_binary)
        
        cv2.imshow("VideoFrame", frame)
        
        if cv2.waitKey(1) > 0:
            break

finally:
    setspeed(0, 0)
    GPIO.cleanup()
    print("finally")

cap.release()
cv2.destroyAllWindows()
print("finish")
