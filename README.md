# Object-Following-Robot-using-OpenCV

https://github.com/Sairaj2807/Object-Following-Robot-using-OpenCV/assets/116910851/9fdb7ab6-8992-4313-a39b-a326024ce852

# Object Following Robot using OpenCV and Arduino

## Project Overview
This project involves creating a robot that can follow a blue-colored ball using OpenCV for image processing and an Arduino microcontroller for motor control. The robot uses a camera to capture images, processes these images to detect and track the blue ball, and sends commands to the Arduino to move towards the ball.

## Components
1. **Hardware**
   - Arduino microcontroller (e.g., Arduino Uno)
   - DC motors and motor driver (e.g.,Cytron motor driver )
   - USB camera
   - Robot chassis with wheels(triomni)
   - Power supply for motors and Arduino
   - Connecting wires

2. **Software**
   - Python 3
   - OpenCV
   - NumPy
   - Pandas
   - pySerial
   - arduino ide

## Installation

### Arduino Setup
1. **Install Arduino IDE**: Download and install the Arduino IDE from [Arduino's official website](https://www.arduino.cc/en/software).
2. **Arduino Code**: Upload the following code to your Arduino board. This code listens for serial commands and controls the motors accordingly.
   ```cpp
   #include "CytronMotorDriver.h"
#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include <SPI.h>
#include "hidjoystickrptparser.h"

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

CytronMD motor1(PWM_DIR, 3, 30);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 5, 31);
CytronMD motor3(PWM_DIR, 6, 32);

// Timers
unsigned long timer = 0;
float timeStep = 0.0261;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
volatile  float yaw = 0;
int count=0;
float kp = 0;
float ki = 0;
float kd =0;
float setpoint = 0;
float p;
float i;
float d;
float error = 0;
float preverror = 0;
int output;
int output1;
int flag_f, flag_b;
float r=0;
int m1;
int m2;
int m3;
float theta;
int ref=0;
int cnt=0;
int ref_red=0;
int ref_blue=0;
int ref_r1=0;
int ref_l1=0;
int ref_r2=0;
int ref_l2=0;

void setup() {

  Serial.begin(9600);


  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  // put your setup code here, to run once:
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);


  //Serial.begin(115200);


#if !defined(MIPSEL)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  Serial.println("Start");

  while (Usb.Init() == -1)
    Serial.println("OSC did not start.");

  delay(200);

  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);
}

void loop() {
  // put your main code here, to run repeatedly:

  timer = millis();


  Vector norm = mpu.readNormalizeGyro();
  yaw = yaw + norm.ZAxis * timeStep;
  Serial.print(" Yaw = ");
  Serial.print(yaw);
  // delay((timeStep * 1000) - (millis() - timer));
kp = 10;
ki = 0;
 kd = 35;

  error = setpoint - yaw;

  p = error;
  i = i + error;
  d = error - preverror;

  output = kp * p + ki * i + kd * d;
  preverror = error;





  Usb.Task();
float yellow = Joy.yellow;
float rxa = JoyEvents.rx;          //         map(JoyEvents.rx, 0, 127, 0, 1023); // map(JoyEvents.Z2, 0, 0xFF, 0.f, 255.f);
float rya = JoyEvents.ry;
float lxa =  JoyEvents.lx;        //  map( JoyEvents.lx, 0, 127,0, 1023);      //map(JoyEvents.Y, 0, 0xFF, 0.f, 255.f);
float lya = JoyEvents.ly;  
float leftjoy = Joy.jl;
float rightjoy = Joy.jr;
float L2 = Joy.lt;
float R2 = Joy.rt;
float back = Joy.bk;
float start = Joy.st;
float red = Joy.red;
float blue = Joy.blue;
float green = Joy.green;
//float yellow = Joy.yellow;
float L1 = Joy.lb;
float R1 = Joy.rb;
float gpad = JoyEvents.ht;

if(blue==0 && red==0){

if(R1==1 && ref_r1==0 && cnt%2==0 ){
  setpoint+=90;
  ref_r1=1;
}
ref_r1=R1;


if(L1==1 && ref_l1==0 && cnt%2==0){
  setpoint-=90;
  ref_l1=1;
}
ref_l1=L1;
}

if(blue==1){
if(L1==1 && ref_l1==0 && cnt%2==0){
  setpoint-=180;
  
  ref_l1=1;
}
ref_l1=L1;
if(R1==1 && ref_r1==0 && cnt%2==0 ){
  setpoint+=180;
  ref_r1=1;
}
ref_r1=R1; 
}

if(red==1){
if(L1==1 && ref_l1==0 && cnt%2==0){
  setpoint-=360;
  
  ref_l1=1;
}
ref_l1=L1;
if(R1==1 && ref_r1==0 && cnt%2==0 ){
  setpoint+=360;
  ref_r1=1;
}
ref_r1=R1; 
}




 Serial.print("  Setpouint ");
 Serial.print(setpoint);


//  *************************************cal of r and angle *************************88
  rxa =  rxa - 128 ;
  rya = 127 - rya;
    lxa =  lxa - 128 ;
 lya = 127 - lya;

  if (rxa > -20  && rxa < 20 ) {
    rxa = 0;
  }
  if (rya > -20 && rya < 20) {
    rya = 0;
  }
  if (lxa > -20 && lxa < 20 ) {
    lxa = 0;
  }
  if (lya > -20 && lya < 20) {
    lya = 0;
  }

  float theta_r = atan2(rya, rxa);
  theta_r = theta_r * 180 / 3.14;
  
 float  theta_l = atan2(lya, lxa);
  theta_l= theta_l * 180 / 3.14;
Serial.print(" | ang_r = ");
Serial.print(theta_r);
Serial.print(" | ang_l ");
Serial.print(theta_l);

  // ***********************************************field************************************

  if(rightjoy==1){
    Serial.print("*********");

  theta_r = theta_r - setpoint ;
  }

if(yellow==1){

theta_r=90;
rxa=0;
rya=127;
}

//

if(green==1){
//  M
rxa=0;
rya=-128;
theta_r=-91;
}

 float  rr = sqrt((rxa * rxa) + (rya * rya));
 rr=map(rr, 0, 130, 0, 91);
 float  rl = sqrt((lxa * lxa) + (lya * lya));
 rl=map(rl, 0, 130, 0, 40);
//  rl = 0;
   r = rr + rl ;

  float tr1 = ((90 - theta_r) * 3.14 / 180);
  float tr2 = ((330 - theta_r) * 3.14 / 180);
  float tr3 = ((210 - theta_r) * 3.14 / 180);

  float tl1 = ((90 - theta_l) * 3.14 / 180);
  float tl2 = ((330 - theta_l) * 3.14 / 180);
  float tl3 = ((210 - theta_l) * 3.14 / 180);

  float vr1 = rr * sin(tr1);
  float vr2 = rr * sin(tr2);
  float vr3 = rr * sin(tr3);
  
  float vl1 = rl * sin(tl1);
  float vl2 = rl * sin(tl2);
  float vl3 = rl * sin(tl3);
float v1 = vr1 + vl1 ;
float v2 = vr2 +  vl2 ;
float v3 = vr3 + vl3 ;

  m1 = v1 - output;
  m2 = v2 - output;
  m3 = v3 - output;
//  
//if(L2==1){
//  setpoint++;
//  
//}
//if(R2==1){
//  setpoint--;
//}

// **************************************************continous rotation ********************************************* 

if(L2==1){
  m1=60;
  m2=60;
  m3=60;
 kp =0;
 ki = 0;
 kd =0;
  setpoint=yaw;
  
}
 if(R2==1){
  m1=-60;
  m2=-60;
  m3=-60;
 kp =0;
 ki = 0;
 kd =0;
  setpoint=yaw;
}
//****************************************MOTOR SPEED ASSIGN************************************************** 
m1 = map(m1, -130,130,-240 , 240);
m2 = map(m2 , -130,130,-240,240);
m3 = map(m3 , -130,130,-240,240);

  m1 = constrain(m1, -235, 235);
  m2 = constrain(m2, -235, 235);
  m3 = constrain(m3, -235, 235);

  motor1.setSpeed(m1);
  motor2.setSpeed(m2);
  motor3.setSpeed(m3);


Serial.print(" | red= ");
Serial.print(red);
Serial.print(" | blue= ");
Serial.print(blue);
Serial.print(" | error= ");
Serial.print(error);
Serial.print(" | v1= ");
Serial.print(m1);
Serial.print(" | v2= ");
Serial.print(m2);
Serial.print(" | v3= ");
Serial.print(m3);

  Serial.print("  right X: ");
  Serial.print(rxa);
  Serial.print("   right Y ");
  Serial.print(rya);
  Serial.print("  left X: ");
  Serial.print(lxa);
  Serial.print("   left Y  ");
  Serial.print(lya);
  Serial.print(" | theta= ");
  Serial.print(theta);
  Serial.print(" | L1= ");
  Serial.print(L1);
  Serial.print(" | r1= ");
  Serial.print(R1);
  Serial.print(" | L2= ");
  Serial.print(L2);
  Serial.print(" | r2= ");
  Serial.print(R2);
  Serial.print(" | Leftjoy = ");
  Serial.print(leftjoy);
  Serial.print(" | rightjoy= ");
  Serial.print(rightjoy);
  Serial.print(" | back= ");
  Serial.print(back);
  Serial.print(" | start= ");
  Serial.print(start);
  Serial.print(" | gpad=");
  Serial.print(gpad);
  Serial.print(" out =");
  Serial.println(output);





}

   ```

### Python Environment Setup
1. **Install Python 3**: Ensure you have Python 3 installed on your system. You can download it from [Python's official website](https://www.python.org/downloads/).
2. **Install Required Libraries**: Use pip to install the necessary libraries.
   ```bash
   pip install opencv-python numpy pandas pyserial
   ```

## Python Code
Here is a sample Python script to detect and track the blue ball, and send commands to the Arduino.
```python
import cv2
import numpy as np
import serial
import time
cen_x=0
cen_y=0
# Define the lower and upper bounds of the color you want to detect (in HSV)
#lower_color = np.array([97, 75, 43])
#upper_color = np.array([120, 255, 203])
arduino_port = "com11"  # Change this to match your system
baud_rate = 115200
ser = serial.Serial(arduino_port, baud_rate)
# Wait for the serial connection to initialize
time.sleep(2)
def nothing():
    pass
print("started")
# Initialize the webcam
cap = cv2.VideoCapture(0)  # 0 for default webcam, you can change this if you have multiple cameras
#cv2.namedWindow("Color Adjustment")

while True:
    
    # Read a frame from the webcam
    ret, frame = cap.read()
    frame = cv2.resize(frame,(600,500))
    if not ret:
        break
     
    lower_bound = np.array([89,38,67])
    upper_bound = np.array([101,255,255])
    # Convert the frame to HSV color space
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask that selects the color within the specified range
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    cv2.line(frame, (300,0),(300,500), (0,0,255), 2)
    cv2.line(frame, (0,500),(600,500), (0,0,255), 2)
    cv2.line(frame,(0,350),(600,350),(0,255,255),1)
    # Find contours in the mask to locate the color object
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        # Calculate the bounding box for each contour
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area>500:
            #cv2.drawContours(frame, contours, -1, (255,0,0),2)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)
            
            
            cen_x = x + (w/2)
            cen_y = y + (h/2)
            u = cen_x - 300
            v = 500 - cen_y
            print("x = ",u," y = ",v)
            ser.write(f"{u} {v}\n".encode())
            #x = ser.readline()
            #print(x)
        # Draw a rectangle around the detected object
        
        
       
        
    cv2.imshow("frame",frame)    
    # Display the original frame with rectangles drawn around the detected objects
    #cv2.imshow('Color Detection', frame)
    #cv2.moveWindow('Color Detection', int(cen_x), int(cen_y))
    # here Remeber to use Color Detection instead of frame because frame doesn.t Exist , we pass the name Color Detection in imshow so use the same name for moveWindow
    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
```

## Usage
1. Connect your Arduino to your computer and upload the Arduino code.
2. Connect the camera to your computer.
3. Run the Python script to start the robot. The robot should start following the blue ball.
   ```bash
   python path_to_your_script.py
   ```

## Troubleshooting
- Ensure the Arduino is connected to the correct serial port.
- Adjust the HSV color range for the blue color if the ball is not being detected correctly.
- Check the wiring of the motors and the motor driver if the robot is not moving as expected.

## Contributing
Contributions are welcome! Please feel free to submit a Pull Request.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.

---

By following these instructions, you should be able to set up your object-following robot using OpenCV and Arduino. If you encounter any issues, please check the troubleshooting section or seek help from the community. Happy building!
