
#include <Wire.h>


float roll, pitch, yaw;

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
  
  Serial.begin(57600);

  Wire.setClock(400000);  // set i2C clock speed

  Wire.begin();
  delay(250);  // give mpu time to start

  // start gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

bool blink = false;

void loop() {

  digitalWrite(13, blink? HIGH : LOW);
  blink = !blink;

  getGyro();

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("Yaw: ");
  Serial.println(yaw);




  delay(50);
}
