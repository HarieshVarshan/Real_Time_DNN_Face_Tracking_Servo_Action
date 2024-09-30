import cv2
import RPi.GPIO as GPIO
import numpy as np
from imutils.video import VideoStream
import argparse
import imutils
import time
import math

def stepper(seq_num,direction):
    for i in range(0,1801):
        for pin in range(0,4):
            Pattern_Pin=ControlPin[pin]
            if seq[seq_num][pin]==1:
                 GPIO.output(Pattern_Pin,True)
            else:
                GPIO.output(Pattern_Pin,False)
        seq_num+=direction
        if seq_num >=8:
            seq_num =0
        elif seq_num <0:
            seq_num=7
        time.sleep(0.001)

def servocontrol(angH,angV):
    mod=((10.5/80)*angH)+7.5
    p.ChangeDutyCycle(mod)
    tilt=((10/80)*(-angV))+7.5
    q.ChangeDutyCycle(tilt)
        
def jump(angH,angV):
    ang1=angH
    ang2=angV
    servocontrol(ang1,ang2)
    
    
    
servo=12
tilt_servo=16
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo,GPIO.OUT)
GPIO.setup(tilt_servo,GPIO.OUT)
p=GPIO.PWM(servo,50)
q=GPIO.PWM(tilt_servo,50)
p.start(7.5)
q.start(7.5)

ControlPin=[4,17,27,22]
for pin in ControlPin:
    GPIO.setup(pin,GPIO.OUT)
    GPIO.output(pin,False)

seq=[[1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1],
    [1,0,0,1]]

seq_num=0

ap = argparse.ArgumentParser()
ap.add_argument("-c", "--confidence", type=float, default=0.2,help="minimum probability to filter weak detections")
args = vars(ap.parse_args())

cap = cv2.VideoCapture(0)
_,frame = cap.read(0)
rows,cols,_ = frame.shape

x_mid=int(cols/2)
y_mid=int(rows/2)
x_ref,y_ref=int(cols/2),int(rows/2)
x_ref=int(cols/2)
print(x_ref,y_ref)
x=y=w=h=0
var=1
net = cv2.dnn.readNetFromCaffe("deploy.prototxt.txt","res10_300x300_ssd_iter_140000.caffemodel")

while True:
    _, frame = cap.read(0)
    #frame = cv2.resize(frame, (640, 480))
    frame=cv2.resize(frame,(480,360))
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0,(300, 300), (104.0, 177.0, 123.0))
    #blob = cv2.dnn.blobFromImage(cv2.resize(frame, (200,200)), 1.0,(200,200), (104.0, 177.0, 123.0))
    net.setInput(blob)
    detections = net.forward()
    for i in range(0, detections.shape[2]):
      confidence = detections[0, 0, i, 2]
      
      if confidence < args["confidence"]:
            continue

      box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
      (startX, startY, endX, endY) = box.astype("int")
      if box!=():
         if var==1:
            var=var+1
            direction=1
            stepper(seq_num,direction)
            
      x_mid=int((startX+endX)/2)
      y_mid=int((startY+endY)/2)
      distance = (3.04*3.14*180)/((endX-startX)+(endY-startY)*360)*1000 + 3
      distance=round(distance,2)
      line=((240-int((startX+endX)/2))*7)/(endY-startY)
      line=round(line,2)
      lineh=((180-int((startY+endY)/2))*7)/(endY-startY)
      lineh=round(lineh,2)
      angH=(math.atan(line/distance)*57.2958)
      print(angH)
      angV=(math.atan(lineh/distance)*57.2958)
      print(angV)
      jump(angH,angV)     
      text = "{:.2f}%".format(confidence * 100)
      y = startY - 10 if startY - 10 > 10 else startY + 10
      cv2.rectangle(frame, (startX, startY), (endX, endY),(0, 0, 255), 2)
      cv2.putText(frame, text, (startX, y),cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)

    cv2.imshow("Frame", frame)
        
    #print(x_mid)
    cv2.line(frame, (x_mid, y_mid), (x_mid, y_mid+10), (255, 255, 255), 2)
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)
    if key == 27:
        direction=-1
        stepper(seq_num,direction)
        break

        
                
cap.release()
cv2.destroyAllWindows()


