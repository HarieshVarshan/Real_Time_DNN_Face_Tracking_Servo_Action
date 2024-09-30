import cv2
import RPi.GPIO as GPIO
import time
import math
def stepper(seq_num):
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
cap = cv2.VideoCapture(0)
_,frame = cap.read(0)
rows,cols,_ = frame.shape

x_mid=int(cols/2)
y_mid=int(rows/2)
x_ref,y_ref=int(cols/2),int(rows/2)
x_ref=int(cols/2)
print(x_ref,y_ref)
x=y=w=h=0
classifier1= cv2.CascadeClassifier("haarcascade_frontalface_default.xml.1")
var=1
while True:
    _, frame = cap.read(0)
    #frame = cv2.resize(frame, (640, 480))
    frame = cv2.resize(frame, (480,360))
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    coord1=classifier1.detectMultiScale(gray,1.05,5)    
    for x,y,w,h in coord1:
        frame =cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),5)
        x_mid=int((x+x+w)/2)
        y_mid=int((y+y+h)/2)
        distance = (3.04*3.14*180)/(w+h*360)*1000 + 3
        distance=round(distance,2)
        line=((240-int((x+x+w)/2))*6.5)/w
        line=round(line,2)
        lineh=((180-int((y+y+h)/2))*6.5)/h
        lineh=round(lineh,2)
        angH=(math.atan(line/distance)*57.2958)
        print(angH)
        angV=(math.atan(lineh/distance)*57.2958)
        print(angV)
        jump(angH,angV)
        cv2.line(frame,(240,0),(240,360),(0,255,0),2)
        cv2.line(frame,(240,int((y+y+h)/2)),(int((x+x+w)/2),int((y+y+h)/2)),(255,255,0),2)
        cv2.putText(frame,'Distance = ' + str(abs(distance/2)) + ' Inch', (50,100),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255),2)
        cv2.putText(frame,"Line = " + str(abs(line)) + "Inch", (100,70),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,0),2)
        break
    
    cv2.line(frame, (x_mid, y_mid), (x_mid, y_mid+10), (255, 255, 255), 1)
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)
    if key == 27:
        direction=-1
        stepper(seq_num)
        break
    if coord1!=():  
        if var==1:
            var=var+1
            direction=1
            stepper(seq_num)
    def jump(angH,angV):
        ang1=angH
        ang2=angV
        servocontrol(ang1,ang2)
    #servocontrol(angH)
        
cap.release()
cv2.destroyAllWindows()
