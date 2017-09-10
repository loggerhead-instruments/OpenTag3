// Copyright Loggerhead Instruments, 2017
// David Mann

// OpenTag3 is an underwater motion datalogger
// designed around the ATMEGA328p

#include <SPI.h>
#include <SdFat.h>
#include <avr/sleep.h>
#include <avr/power.h>

// Using SoftWire because Arduino Wire library does not work with KMX62 accel/mag
#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5
#define I2C_TIMEOUT 100
//#define I2C_FASTMODE 1

#include <SoftWire.h>
#include <avr/io.h>
SoftWire Wire = SoftWire();

//
// DEV SETTINGS
//
float codeVer = 1.00;

//
// DEV SETTINGS
//

// pin assignments
#define chipSelect 10
#define LED_GRN 4
#define LED_RED A3
#define BURN 8
#define VHFPOW 9
#define BUTTON1 A2
#define SD_POW 5

// SD file system
SdFat sd;
File dataFile;

// sensor values
int accelX, accelY, accelZ;
int magX, magY, magZ;

void setup() {
  Serial.begin(115200);
  
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BURN, OUTPUT);
  pinMode(VHFPOW, OUTPUT);
  pinMode(BUTTON1, INPUT);
  digitalWrite(BURN,LOW);
  digitalWrite(LED_RED,LOW);
  digitalWrite(LED_GRN,HIGH);
  digitalWrite(VHFPOW, LOW);
  digitalWrite(BURN, LOW);
  pinMode(SD_POW, OUTPUT);      
  digitalWrite(SD_POW, HIGH); 
  delay(2000);
  Serial.println("On");

  Wire.begin();

  Serial.println("Init microSD");
  // see if the card is present and can be initialized:
  while (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    Serial.println("Card failed");
    digitalWrite(LED_RED, HIGH);
    delay(200);
    digitalWrite(LED_RED, LOW);
    delay(100);
  }

  initSensors();

}

void loop() {
  delay(1000);

}


void initSensors(){
  kmx62Init(1); // init with FIFO mode
  kmx62SampleRate(100);
  kmx62ClearFifo();
  kmx62Start();
  Serial.println(kmx62TestResponse());

  for(int x=0; x<100l; x++){
    kmx62Read();
    Serial.print("Accel/Mag ");
    Serial.print(accelX); Serial.print(" ");
    Serial.print(accelY); Serial.print(" ");
    Serial.print(accelZ); Serial.print(" ");
    Serial.print(magX); Serial.print(" ");
    Serial.print(magY); Serial.print(" ");
    Serial.println(magZ);
    delay(1000);
  }
}


