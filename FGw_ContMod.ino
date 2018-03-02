#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

//software SPI
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);

//A fist experiences significant acceleration during a good punch,
//followed by massive deceleration at the end of the punch.
//This happens within a fairly small window of time, so it's
//pretty easy to distinguish a punch from normal gesticulations.

unsigned long punchStart = 0;//variable for non-blocking punch timeframe check
const long punchInterval = 200;//timeframe of a punch in ms, from max acceleration to max deceleration, 200 is very generous
int punchAccel = 20;//the beginning of a punch in m/s^2, could be over 50m/s^2 depending on the puncher
int punchDecel = -40;//the end of a punch in m/s^2, could be less than -100m/s^2 depending on the puncher
int flameTime = 250;//how long the flame lasts, in ms
int buttonState = 0;//state of button

void setup(void) {
  //Test to see if accelerometer is communicating
  Serial.begin(9600);
  Serial.println("LIS3DH test!");
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_16_G);   //+-16G range for good punch detection
  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  pinMode(2, INPUT); //Button
  pinMode(8, OUTPUT); //Solenoid valve
  pinMode(9, OUTPUT); //Arc lighter
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(2, HIGH);
}

void loop() {
  buttonState = digitalRead(2);
  if (buttonState == HIGH) {
  }
  else {
    digitalWrite(8, HIGH);
    digitalWrite(9, HIGH);
    delay(250);
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
  }
  lis.read();
  sensors_event_t event;
  lis.getEvent(&event);

  //look for punch starting, at least 20 m/s^2
  if (event.acceleration.x > punchAccel) {
    Serial.println(event.acceleration.x);
    punchStart = millis();
  }

  unsigned long currentMillis = millis();

  //look for punch ending, less than -40 m/s^2
  if (event.acceleration.x < punchDecel && currentMillis - punchStart < punchInterval) {
    Serial.println(event.acceleration.x);
    Serial.println("Punch");
    Fire(flameTime);
  }
}

void Fire(int flameTime) {
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  delay(flameTime);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
}

