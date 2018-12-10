/* Code to view on the Serial Monitor the values of the Gyroscope
 * and the output for the mouse. This is equal to the standard code
 * but with instructions to check the values on the Serial monitor.
 * Change the delay to read the values at reduced speed.
 * 
 * Gabry295
 */

#include <Wire.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include <MPU6050.h>
#include <Mouse.h>

MPU6050 mpu;
#define INTERRUPT_PIN 7
int16_t ax, ay, az, gx, gy, gz;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int MPUOffsets[6] = {  -1836,  -3755,  3327,  43,  4,  94}; //Run MPU6050_Calibration to obtain these
//int vx, vy;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// set pin numbers for switch, and LED:
const int switchPin = 10;      // switch to turn on and off mouse control
//const int ledPin = 13;         // Mouse control LED
//variables to turn on/off the mouse
bool mouseIsActive = false;    // whether or not to control the mouse
int lastSwitchState = HIGH;        // previous switch state



// Configuration values to adjust the sensitivity and speed of the mouse.
// X axis (left/right) configuration:
#define XACCEL_MIN 1      // Minimum range of X axis acceleration, values below
                            // this won't move the mouse at all.
#define XACCEL_MAX 30      // Maximum range of X axis acceleration, values above
                            // this will move the mouse as fast as possible.
#define XMOUSE_RANGE 120   // Range of velocity for mouse movements.  The higher
                            // this value the faster the mouse will move.
#define XMOUSE_SCALE 1      // Scaling value to apply to mouse movement, this is
                            // useful to set to -1 to flip the X axis movement.

// Y axis (up/down) configuration:
// Note that the meaning of these values is exactly the same as the X axis above,
// just applied to the Y axis and up/down mouse movement.  You probably want to
// keep these values the same as for the X axis (which is the default, they just
// read the X axis values but you can override with custom values).
#define YACCEL_MIN XACCEL_MIN
#define YACCEL_MAX XACCEL_MAX
#define YMOUSE_RANGE XMOUSE_RANGE
#define YMOUSE_SCALE 1

// these constants won't change:
const int knockSensor = A0; // the piezo is connected to analog pin 0
const int threshold = 100;  // threshold value to decide when the detected sound is a knock or not

// these variables will change:
int sensorReading = 0;      // variable to store the value read from the sensor pin

// Floating point linear interpolation function that takes a value inside one
// range and maps it to a new value inside another range.  This is used to transform
// each axis of acceleration to mouse velocity/speed. See this page for details
// on the equation: https://en.wikipedia.org/wiki/Linear_interpolation
float lerp(float x, float x0, float x1, float y0, float y1) {
  // Check if the input value (x) is outside its desired range and clamp to
  // those min/max y values.
  if (x <= x0) {
    return y0;
  }
  else if (x >= x1) {
    return y1;
  }
  // Otherwise compute the value y based on x's position within its range and
  // the desired y min & max.
  return y0 + (y1-y0)*((x-x0)/(x1-x0));
}

float accel_motion(int16_t a){
    float i = a/16380; //2g values
    return (i * 9.80665F); //m/s^2
    }

int int_lerp(int x, int  x0, int x1, int y0, int y1){
    if (x <= x0){
      return y0;
    }else if( x >= x1){
      return y1;  
    }
    return y0 + (y1-y0)*((x-x0)/(x1-x0));
  }

int gyro_motion(int16_t g){
    int j = g/131;  //-+250 degrees/second
    return j;
  }

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    //Serial.begin(115200);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(MPUOffsets[0]);
    mpu.setYAccelOffset(MPUOffsets[1]);
    mpu.setZAccelOffset(MPUOffsets[2]);
    mpu.setXGyroOffset(MPUOffsets[3]);
    mpu.setYGyroOffset(MPUOffsets[4]);
    mpu.setZGyroOffset(MPUOffsets[5]);
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

    // configure LED for output
    //pinMode(switchPin, OUTPUT);
    digitalWrite(switchPin, HIGH);
    Mouse.begin();
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read the switch:
  int switchState = digitalRead(switchPin);
  // if it's changed and it's high, toggle the mouse state:
  if (switchState != lastSwitchState) {
    if (switchState == LOW) {
      mouseIsActive = !mouseIsActive;
      // turn on LED to indicate mouse state:
      //digitalWrite(ledPin, mouseIsActive);
      //Serial.println("Mouse active");
    }
  }
  // save switch state for next comparison:
  lastSwitchState = switchState;

  //the follow does the right-click of the mouse, accordingly with whats is measured on the sensor
  sensorReading = analogRead(knockSensor);
  // if the sensor reading is greater than the threshold:
  if (sensorReading >= threshold) {
    // toggle the status of the ledPin:
    //ledState = !ledState;
    // update the LED pin itself:
    //digitalWrite(ledPin, ledState);
    // send the string "Knock!" back to the computer, followed by newline
    // if the mouse-click is not pressed, press it:
    if (!Mouse.isPressed(MOUSE_LEFT)) {
      Mouse.press(MOUSE_LEFT);
      //Serial.println("Pressed!");
       
    }
    //Serial.println("Pressed!");
  }
  else {
    // if the mouse is pressed, release it:
    if (Mouse.isPressed(MOUSE_LEFT)) {
      Mouse.release(MOUSE_LEFT);
      //Serial.println("End of click!");
    }
  }

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Grab x, y gyro values (in degrees per second).
  int x = gyro_motion(gy);
  int y = gyro_motion(gz);
  // Use the magnitude of acceleration/gyro to interpolate the mouse velocity.
  int x_mag = abs(x);
  int x_mouse = lerp(x_mag, XACCEL_MIN, XACCEL_MAX, 0, XMOUSE_RANGE);
  int y_mag = abs(y);
  int y_mouse = lerp(y_mag, YACCEL_MIN, YACCEL_MAX, 0, YMOUSE_RANGE);
  // Change the mouse direction based on the direction of the acceleration.
  if (x < 0) {
    x_mouse *= -1;
  }
  if (y < 0) {
    y_mouse *= -1;
  }
  // Apply any global scaling to the axis (to flip it for example) and truncate
  // to an integer value.
  //x_mouse = floor(x_mouse*XMOUSE_SCALE);
  //y_mouse = floor(y_mouse*YMOUSE_SCALE);
  
  /*double mouseX=(gx)*(ax) +(gy)*(ay);
  double mouseY=(gz);
  
  vx = -(gx)/16384; //return value in m/s^2
  vy = -(gz)/16384; //return value in m/s^2
  //vx = readAxis(gx);
  //vy = readAxis(gz);
  */
  
  //Serial.print("gx = ");
  //Serial.print(gx);
  //Serial.print(" | gy = ");
  //Serial.print(gy);
  //Serial.print(" | gz = ");
  //Serial.print(gz);
  
  //Serial.print("        | X = ");
  //Serial.print(x);
  //Serial.print(" | Y = ");
  //Serial.print(y);
  //Serial.print(" | mouseX = ");
  //Serial.print(x_mouse);
  //Serial.print(" | mouseY = ");
  //Serial.println(y_mouse);
  
  if(mouseIsActive){
    //Serial.println("Mouse ativo");
    Mouse.move(-x_mouse, -y_mouse, 0);
  }

  delay(20);
}
