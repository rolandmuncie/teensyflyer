
#include <Wire.h>
#include <PulsePosition.h>


float roll, pitch, yaw;

float calibratedRoll, calibratedPitch, calibratedYaw;
float inputRoll, inputPitch, inputYaw;

const int ESC_MIN_SIGNAL = 1000;  // Minimum throttle signal (in microseconds)
const int ESC_MAX_SIGNAL = 2000;  // Maximum throttle signal (in microseconds)
const int ESC_ARM_SIGNAL = 1500;  // Mid-point signal, often used for arming

PulsePositionInput ReceiverInput(RISING);  // read PPM pulse from rising edge (not PWM)

float receiver[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int channel = 0;

// PID variables

float desiredRoll, desiredPitch, desiredYaw;
float errorRoll, errorPitch, errorYaw;
float previousErrorRoll = 0, previousErrorPitch = 0, previousErrorYaw = 0;
float previousIRoll = 0, previousIPitch = 0, previousIYaw = 0;  // previous integrated values

float PID[] = { 0, 0, 0 };

// P constants
float const pRoll = .6, pPitch = .6, pYaw = 2;
// I constants
float const iRoll = 3.5, iPitch = 3.5, iYaw = 12;
// D constants
float const dRoll = 0.03, dPitch = 0.03, dYaw = 0;

void pid(float error, float p, float i, float d, float previousError, float previousI) {

  float pTerm = p * error;
  float iTerm = previousI + i * (error + previousError) * 0.004 / 2;  // 0.0.04 is the time interval of 4 ms (because 250Hz interval)

  // keep I in the the range of -400 to +400 to avoid integral windup
  if (iTerm > 400) iTerm = 400;
  else if (iTerm < -400) iTerm = -400;

  float dTerm = d * (error + previousError) / 0.004;
  float pidOutput = pTerm + iTerm + dTerm;  // todo: when I did this the last time with Igor, we didn't use D. Why???

  if (pidOutput > pidOutput) iTerm = 400;
  else if (pidOutput < -400) pidOutput = -400;

  PID[0] = pidOutput;
  PID[1] = error;
  PID[2] = iTerm;
}

void getReceiver(void) {

  // read all receiver values
  channel = ReceiverInput.available();
  if (channel > 0) {
    for (int c = 1; c <= channel; c++) {
      receiver[c - 1] = ReceiverInput.read(c);
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

  pinMode(13, OUTPUT);      // on board LED
  ReceiverInput.begin(14);  // pin 14 connected to receiver channel 4

  digitalWrite(13, HIGH);  //turn on on board LED during setup. during opertation it will blink

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
  for (int i = 0; i < 2000; i++) {
    getGyro();

    calibratedRoll += roll;
    calibratedPitch += pitch;
    calibratedYaw += yaw;
    delay(1);
  }

  calibratedRoll /= 2000;
  calibratedPitch /= 2000;
  calibratedYaw /= 2000;

  // PWM to motor setup on pin 1
  analogWriteFrequency(1, 250);  // pwm for motor on pin 1, 250Hz
  analogWriteResolution(12);     //  12 bit signal between 0 and 4095  ( default 8 bit would only give us 0 to 255)

  // note to self: 0 = 0 micro secs, 4095 = 4000 microsecs .. so need to multiply with 4095/4000 = 1.0214 when setting PWM
}

bool blink = false;

void loop() {

  digitalWrite(13, blink ? HIGH : LOW);
  blink = !blink;

  getGyro();

  // adjust values with calibration values
  roll -= calibratedRoll;
  pitch -= calibratedPitch;
  yaw -= calibratedYaw;



  /*Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("Yaw: ");
  Serial.println(yaw); */

  getReceiver();

  desiredRoll = 0.15 * (receiver[0]-1500);
  desiredPitch = 0.15 * (receiver[1]-1500);
  desiredYaw = 0.15 * (receiver[3]-1500);
  float throttle = receiver[2];

  errorRoll = desiredRoll-roll;
  errorPitch = desiredPitch - pitch;
  errorYaw = desiredYaw - yaw;

  pid(errorRoll, pRoll, iRoll, dRoll, previousErrorRoll, previousIRoll);
  inputRoll = PID[0];
  previousErrorRoll = PID[1];
  previousIRoll = PID[2];

  pid(errorPitch, pPitch, iPitch, dPitch, previousErrorPitch, previousIPitch);
  inputPitch = PID[0];
  previousErrorPitch = PID[1];
  previousIPitch = PID[2];

  pid(errorYaw, pYaw, iYaw, dYaw, previousErrorYaw, previousIYaw);
  inputYaw = PID[0];
  previousErrorYaw = PID[1];
  previousIYaw = PID[2];
 

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










// motor1: counterclockwise right front
// motor2: clockwise right rear
// motor3: counterclockwise left rear
// motor4: clockwise left front

// calculate adjusted motor values and multiply by 1.024 to convert from ms to 12 bit equivalent
float motor1 = 1.024 * (throttle - inputRoll - inputPitch - inputYaw);
 float motor2 = 1.024 * (throttle - inputRoll + inputPitch + inputYaw);
  float motor3 = 1.024 * (throttle + inputRoll + inputPitch - inputYaw);
  float motor4 = 1.024 * (throttle + inputRoll - inputPitch + inputYaw); 

  // cap motor output ar 2000ms
  if (motor1 > 2000) motor1 = 2000;
  if (motor2 > 2000) motor2 = 2000;
  if (motor3 > 2000) motor3 = 2000;
  if (motor4 > 2000) motor4 = 2000;

 /* int idle = 1150;  // 15% power to avoid stall

if (motor1 < idle) motor1 = idle;
if (motor2 < idle) motor2 = idle;
if (motor3 < idle) motor3 = idle;
if (motor4 < idle) motor4 = idle;*/
 

// run motor1
analogWrite(1, motor1);


delay(50);
}
