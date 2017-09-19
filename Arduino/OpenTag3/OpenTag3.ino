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
#define I2C_FASTMODE 1

#include <SoftI2CMaster.h>
#include <avr/io.h>

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
int fileCount;

// sensor values
int accelX, accelY, accelZ;
int magX, magY, magZ;
#define BUF_BYTES 96
uint8_t fifoVal[BUF_BYTES];

long startTime;

void setup() {
  Serial.begin(115200);
  Serial.println("On");
  delay(1000);

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
  
  byte I2C_check = i2c_init();
  if(I2C_check == false){
    Serial.println("I2C Init Failed--SDA or SCL may not be pulled up!");
     while(1){
       digitalWrite(LED_RED, HIGH);
       delay(500);
       digitalWrite(LED_RED, LOW);
       delay(500);
     }
  }

  Serial.println("Init microSD");
  // see if the card is present and can be initialized:
  while (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    Serial.println("Card failed");
    digitalWrite(LED_RED, HIGH);
    delay(200);
    digitalWrite(LED_RED, LOW);
    delay(100);
  }

  fileInit();
  initSensors();
  startTime = millis();

}



void loop() {
  int fifoPts = kmx62GetFifoPoints();
  if(fifoPts>BUF_BYTES * 2){
//    Serial.print(millis());
//    Serial.print(" ");
//    Serial.print(millis() - startTime);
//    Serial.print(" ");
//    Serial.print(fifoPts);
//    Serial.print(" ");
    kmx62FifoRead();
    dataFile.write(fifoVal, BUF_BYTES);
    kmx62FifoRead();
    dataFile.write(fifoVal, BUF_BYTES);
//    Serial.println(fifoVal[1]<<8 | fifoVal[0]);
//    startTime = millis();
  }
  if(millis() - startTime>10000){
    dataFile.close();
    Serial.println("Test done");
    digitalWrite(LED_RED, HIGH);
    while(1);
  }
}

void initSensors(){
  int testResponse = kmx62TestResponse();
  while(testResponse!=85){
    Serial.print("KMX not recognized: ");
    Serial.println(testResponse);
    delay(500);
    digitalWrite(LED_RED, HIGH);
    kmx62Reset();
    testResponse = kmx62TestResponse();
  }
  digitalWrite(LED_RED, LOW);
  kmx62Init(1); // init with FIFO mode
  kmx62SampleRate(1600);
  
  kmx62Start(0x5F);
  kmx62ClearFifo();
  
  for(int x=0; x<100; x++){
    kmx62Read();
    Serial.print("Accel/Mag ");
    Serial.print(accelX); Serial.print(" ");
    Serial.print(accelY); Serial.print(" ");
    Serial.print(accelZ); Serial.print(" ");
    Serial.print(magX); Serial.print(" ");
    Serial.print(magY); Serial.print(" ");
    Serial.println(magZ);
    Serial.print("Fifo:");
    Serial.println(kmx62GetFifoPoints());
    delay(20);
  }

  kmx62ClearFifo();
}

void fileInit()
{
   char filename[13];
   sprintf(filename,"test.16");
   dataFile = sd.open(filename, O_WRITE | O_CREAT);
   while (!dataFile){
    fileCount += 1;
    sprintf(filename,"F%06d.amx",fileCount); //if can't open just use count
    dataFile = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
    Serial.println(filename);
   }
}
