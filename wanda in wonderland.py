import cv2
import numpy as np
from controller import Robot, Motor, DistanceSensor

robot = Robot()
timestep = int(robot.getBasicTimeStep())

rightmotor = robot.getDevice('right wheel motor')
leftmotor = robot.getDevice('left wheel motor')

rightmotor.setPosition(float('inf'))
leftmotor.setPosition(float('inf'))

rightmotor.setVelocity(0.0)
leftmotor.setVelocity(0.0)



camera = robot.getDevice('camera')
camera.enable(timestep)
cap = camera.getImageArray()

P= 0.0048
max_speed = 6.27
Max_vel = 6.27

prox_sensor = []


for i in range(8):
    name = 'ps' + str(i)
    prox_sensor.append(robot.getDevice(name))
    prox_sensor[i].enable(timestep)

def leftturn(speed):
    if speed > 100:
        speed = 100
    speed = (max_speed / 100.0) * speed
    for _ in range(10):
        leftmotor.setVelocity(speed / 8)
        rightmotor.setVelocity(speed / 2)

def right(speed):
    if speed > 100:
        speed = 100
    speed = (max_speed / 100.0) * speed
    for _ in range(10):
        leftmotor.setVelocity(speed)
        rightmotor.setVelocity(-speed)


def forward(speed):
    if speed > 100:
        speed = 100
    speed = (max_speed / 100.0) * speed
    for _ in range(10):
        leftmotor.setVelocity(speed)
        rightmotor.setVelocity(speed)


def left(speed):
    if speed > 100:
        speed = 100
    speed = (max_speed / 100.0) * speed
    for _ in range(10):
        leftmotor.setVelocity(-speed)
        rightmotor.setVelocity(speed)


def rightturn(speed):
    if speed > 100:
        speed = 100
    speed = (max_speed / 100.0) * speed
    for _ in range(10):
        leftmotor.setVelocity(speed / 2)
        rightmotor.setVelocity(speed / 8)


def stop():
    leftmotor.setVelocity(0)
    rightmotor.setVelocity(0)

def leftIrReading():
    return prox_sensor[5].getValue()
    
def frontIrReading():
    return (prox_sensor[7].getValue() + prox_sensor[0].getValue()) / 2

def leftcIrReading():
    return prox_sensor[6].getValue()


def rightcIrReading():
    return prox_sensor[1].getValue()


def get_image_from_camera():
    img = camera.getImageArray()
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    return cv2.flip(img, 1)


def green1(img2):
    lower_color = np.array([25, 52, 72])
    upper_color = np.array([102, 255, 255])
    img2 = cv2.medianBlur(img2, 3)
    mask_color = cv2.inRange(img2, lower_color, upper_color)

    det_color = cv2.bitwise_and(img, img, mask=mask_color)
    contours, garb = cv2.findContours(mask_color, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if (len(contours) >= 1):
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
        if (cv2.contourArea(contours[0]) > 500):
            M = cv2.moments(contours[0])
            if (M['m00'] != 0):
                cxg = int(M['m10'] / M['m00'])
                return cxg
            else: 
                return 0                
        else: 
            return 0
    else: 
        return 0
def green2(cxg):
    error = camera.getWidth() / 2 - cxg
    leftmotor.setVelocity( error * P * Max_vel)
    rightmotor.setVelocity(-1* error * P * Max_vel)
    if (error < 5 and error >-5):
        forward(100)
        if (frontIrReading() > 80):
            right(100)
        else:
            if (leftIrReading() > 60):
                forward(100)
            else:
                leftturn(100)
            if (leftcIrReading() > 80):
                rightturn(100)

def blue1(img2):
    lower_color = np.array([94, 80, 2])
    upper_color = np.array([126, 255, 255])
    img2 = cv2.medianBlur(img2, 3)
    mask_color = cv2.inRange(img2, lower_color, upper_color)

    det_color = cv2.bitwise_and(img, img, mask=mask_color)
    contours, garb = cv2.findContours(mask_color, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if (len(contours) >= 1):
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
        if (cv2.contourArea(contours[0]) > 500):
            M = cv2.moments(contours[0])
            if (M['m00'] != 0):
                cxb = int(M['m10'] / M['m00'])
                return cxb
            else: 
                return 0                
        else: 
            return 0
    else: 
        return 0
def blue2(cxb):
    error = camera.getWidth() / 2 - cxb
    leftmotor.setVelocity(- error * P * Max_vel)
    rightmotor.setVelocity(error * P * Max_vel)
    if (error < 10):
        forward(100)
        if (frontIrReading() > 80):
            right(100)
        else:
            if (leftIrReading() > 60):
                forward(100)
            else:
                leftturn(100)
            if (leftcIrReading() > 80):
                rightturn(100)


def purple1(img2):
    lower_color = np.array([80, 10, 10])
    upper_color = np.array([120, 255, 255])
    img2 = cv2.medianBlur(img2, 3)
    mask_color = cv2.inRange(img2, lower_color, upper_color)

    det_color = cv2.bitwise_and(img, img, mask=mask_color)
    contours, garb = cv2.findContours(mask_color, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
  
    if (len(contours) >= 1):
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
        if (cv2.contourArea(contours[0]) > 500):
            M = cv2.moments(contours[0])
            if (M['m00'] != 0):
                cxp = int(M['m10'] / M['m00'])
                return cxp
            else: 
                return 0                
        else: 
            return 0
    else: 
        return 0
def purple2(cxp):
    error = camera.getWidth() / 2 - cxp
    leftmotor.setVelocity(- error * P * Max_vel)
    rightmotor.setVelocity(error * P* Max_vel)
    if (error < 10):
        forward(100)
        if (frontIrReading() > 80):
            right(100)
        else:
            if (leftIrReading() > 60):
                forward(100)
            else:
                leftturn(100)
            if (leftcIrReading() > 80):
                rightturn(100)


def red1(img2):
    lower_color = np.array([161, 155, 84])
    upper_color = np.array([179, 255, 255])
    img2 = cv2.medianBlur(img2, 3)
    mask_color = cv2.inRange(img2, lower_color, upper_color)

    det_color = cv2.bitwise_and(img, img, mask=mask_color)
    contours, garb = cv2.findContours(mask_color, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    if (len(contours) >= 1):
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
        if (cv2.contourArea(contours[0]) > 500):
            M = cv2.moments(contours[0])
            if (M['m00'] != 0):
                cxr = int(M['m10'] / M['m00'])
                return cxr
            else: 
                return 0                
        else: 
            return 0
    else: 
        return 0

def red2(cxr):
    error = camera.getWidth() / 2 - cxr
    left_motor.setVelocity(- error * P * Max_vel)
    right_motor.setVelocity(error * P * Max_vel)
    if (error < 10):
        forward(100)
        if (frontIrReading() > 80):
            stop()

count = 0
while robot.step(timestep) != -1:
    img = get_image_from_camera()
    img = cv2.resize(img, (400, 400))
    img2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    g = green1(img2)
    b = blue1(img2)
    r = red1(img2)
    p = purple1(img2)
    
    if (g < 300 and g != 0):
        green2(green1(img2))
        if (frontIrReading() > 80):
            count = 1
    if (g > -300 and g != 0):
        green2(green1(img2))
        if (frontIrReading() > 80):
            count = 1
    if (b < 300 and b != 0):
        if (count == 1):
            blue2(blue1(img2))
            if (frontIrReading() > 80):
                count = 2
    if (b > -300 and b != 0):
        if (count == 1):
            blue2(blue1(img2))
            if (frontIrReading() > 80):
                count = 2
    if (p < 300 and p != 0):
        if (count == 2):
            purple2(purple1(img2))
            if (frontIrReading() > 80):
                count = 3
    if (p > -300 and p != 0):
        if (count == 2):
            purple2(purple1(img2))
            if (frontIrReading() > 80):
                count = 3
    if (r < 300 and r != 0):
        if (count == 3):
            red2(red1(img2))
            if (frontIrReading() > 80):
                count = 4
    if (r > -300 and r != 0):
        if (count == 3):
            red2(red1(img2))
            if (frontIrReading() > 80):
                count = 4
    else:
        if (frontIrReading() > 80):
            right(100)
        else:
            if (leftIrReading() > 60):
                forward(100)
            else:
                leftturn(100)
            if (leftcIrReading() > 80):
                rightturn(100)
