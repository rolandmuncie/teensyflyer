
#include <Wire.h>
#include <PulsePosition.h>


float roll, pitch, yaw;

float calibratedRoll, calibratedPitch, calibratedYaw;

const int ESC_MIN_SIGNAL = 1000;  // Minimum throttle signal (in microseconds)
const int ESC_MAX_SIGNAL = 2000;  // Maximum throttle signal (in microseconds)
const int ESC_ARM_SIGNAL = 1500;  // Mid-point signal, often used for arming

PulsePositionInput ReceiverInput(RISING);  // read PPM pulse from rising edge (not PWM)

float receiver[]={0,0,0,0,0,0,0,0};
int channel=0;

 void getReceiver(void) {

      // read all receiver values
      channel = ReceiverInput.available();
      if (channel >0) {
        for (int c=1;c<=channel;c++) {
          receiver[c-1]=ReceiverInput.read(c);
        }
      }

 }


void getGyro(void) {


  Wire.beginTransmission(0x68);  //  gyro register
  Wire.write(0x1A);              // low pass filter on
  Wire.write(0x05);              // 10 Hz, to filter out motor vibrations
  Wire.endTransmission();

  Wire.beginTransmission(0x68);  //  gyro register
  Wire.write(0x1B);              // Sensitivity scale factor
  Wire.write(0x8);               // 65.5 LSB/degree/sec
  Wire.endTransmission();

  Wire.beginTransmission(0x68);  //  gyro register
  Wire.write(0x43);              // request gyro measurements
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);  // request 6 byte

  // values are 16-bit 2's complement
  int16_t x = Wire.read() << 8 | Wire.read();  // unsigned 16 bit int
  int16_t y = Wire.read() << 8 | Wire.read();  // unsigned 16 bit int
  int16_t z = Wire.read() << 8 | Wire.read();  // unsigned 16 bit int

  // results are in LSB, to convert to degree/sec divide by 65.5

  roll = (float)x / 65.5;
  pitch = (float)y / 65.5;
  yaw = (float)z / 65.5;
}

void setup() {

  pinMode(13, OUTPUT);  // on board LED
  ReceiverInput.begin(14); // pin 14 connected to receiver channel 4
  
  Serial.begin(57600);

  Wire.setClock(400000);  // set i2C clock speed

  Wire.begin();
  delay(250);  // give mpu time to start

  // start gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // calibrate
  // basically read out a large number of values when first starting and average them out
  for (int i=0; i <2000; i++) {
    getGyro();

    calibratedRoll+=roll;
    calibratedPitch+=pitch;
    calibratedYaw+=yaw;
    delay(1);
  }

    calibratedRoll/=2000;
    calibratedPitch/=2000;
    calibratedYaw/=2000;

  // PWM to motor setup on pin 1
    analogWriteFrequency(1,250);   // pwm for motor on pin 1, 250Hz 
    analogWriteResolution(12);  //  12 bit signal between 0 and 4095  ( default 8 bit would only give us 0 to 255)

    // note to self: 0 = 0 micro secs, 4095 = 4000 microsecs .. so need to multiply with 4095/4000 = 1.0214 when setting PWM

    

}

bool blink = false;

void loop() {

  digitalWrite(13, blink? HIGH : LOW);
  blink = !blink;

  getGyro();

  // adjust values with calibration values
  roll-=calibratedRoll;
  pitch-=calibratedPitch;
  yaw-=calibratedYaw;

  /*Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("Yaw: ");
  Serial.println(yaw); */

  getReceiver();

  // receiver values shouod be between 1000 and 2000 microseconds
  Serial.print("Channels:");
  Serial.print(channel);
  Serial.print(" Roll:");
  Serial.print(receiver[0]);
  Serial.print(" Pitch:");
  Serial.print(receiver[1]);
  Serial.print(" Throttle:");
  Serial.print(receiver[2]);
  Serial.print(" Yaw:");
  Serial.println(receiver[3]);




  float throttle = receiver[2] * 1.024;

  

  

  // motor1: counterclockwise right front
  // motor2: clockwise right rear
  // motor3: counterclockwise left rear
  // motor4: clockwise left front
  
  float motor1 = throttle - roll - pitch - yaw;
 /* float motor2 = throttle - roll + pitch + yaw;
  float motor3 = throttle + roll + pitch - yaw;
  float motor4 = throttle + roll - pitch + yaw; */

// run motor1 
  analogWrite(1, motor1) ;


  delay(50);
}
