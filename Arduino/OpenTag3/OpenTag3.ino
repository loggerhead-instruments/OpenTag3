// Copyright Loggerhead Instruments, 2017
// David Mann

// OpenTag3 is an underwater motion datalogger
// designed around the ATMEGA328p

// OpenTag3 supports the following sensors:
// Pressure/Temperature
// acclerometer/magnetometer
// RGB light


/*
 *  WDT
 *  check for errors on sensor init
 */


//#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <MsTimer2.h> 
#include <avr/sleep.h>
#include <avr/power.h>
#include <prescaler.h>

#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5

#define I2C_TIMEOUT 100
#define I2C_FASTMODE 1

#include <SoftWire.h>
#include <avr/io.h>
#include <avr/boot.h>

SoftWire Wire = SoftWire();

//
// DEV SETTINGS
//
char codeVer[12] = "2018-04-04";
int printDiags = 1;

int recDur = 600; // 3600 seconds per hour
int recInt = 0;
int LED_EN = 1; //enable green LEDs flash 1x per pressure read. Can be disabled from script.

#define HALL_EN 0 //flash red LED for Hall sensor

#define MS5837_30bar // Pressure sensor. Each sensor has different constants.

#ifdef MS5837_02bar
  #define MS58xx_constant 327680.0
  #define pressAddress 0x76
#endif
#ifdef MS5837_30bar
  #define MS58xx_constant 8192.0
  #define pressAddress 0x76
#endif

// pin assignments
#define chipSelect  10
#define LED_GRN A3  // PD4
#define LED_RED 4 // PC3
#define BURN 8     // PB0
#define VHFPOW 9   // PB1
#define BUTTON1 A2 // PC2
#define BAT_VOLTAGE A7// ADC7
#define HALL 3 // PD3 (INT1)
#define CAM_TRIG 5 
#define CAM_EN 6  // High enables 5V power boost board

// SD file system
SdFat sd;
File dataFile;
int fileCount; 

int ssCounter; // used to get different sample rates from one timer based on imu_srate
byte clockprescaler=0;  //clock prescaler

//
// SENSORS
//
byte imuTempBuffer[20];
int imuSrate = 50; // must be integer for timer
int sensorSrate = 1; // must divide into imuSrate
int slowRateMultiple = imuSrate / sensorSrate;
int speriod = 1000 / imuSrate;

//Pressure and temp calibration coefficients
uint16_t PSENS; //pressure sensitivity
uint16_t POFF;  //Pressure offset
uint16_t TCSENS; //Temp coefficient of pressure sensitivity
uint16_t TCOFF; //Temp coefficient of pressure offset
uint16_t TREF;  //Ref temperature
uint16_t TEMPSENS; //Temperature sensitivity coefficient
byte Tbuff[3];
byte Pbuff[3];
volatile float depth, temperature, pressure_mbar;
boolean togglePress = 0; // flag to toggle conversion of temperature and pressure

// RGB
int16_t islRed;
int16_t islBlue;
int16_t islGreen;
int16_t accelX, accelY, accelZ;
int16_t magX, magY, magZ;
int16_t gyroX, gyroY, gyroZ;

int accel_scale = 16;

// impeller spin counter
volatile int spin;

// System Modes and Status
int mode = 0; //standby = 0; running = 1
volatile float voltage;

// Time
volatile byte second = 0;
volatile byte minute = 0;
volatile byte hour = 17;
volatile byte day = 1;
volatile byte month = 1;
volatile byte year = 17;

unsigned long t, startTime, endTime, burnTime;
int burnFlag = 0;
long burnSeconds;

void setup() {
  Serial.begin(115200);
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BURN, OUTPUT);
  pinMode(VHFPOW, OUTPUT);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(HALL, INPUT);
  pinMode(CAM_TRIG, OUTPUT);
  pinMode(CAM_EN, OUTPUT);
  
  digitalWrite(BURN,LOW);
  digitalWrite(LED_RED,LOW);
  digitalWrite(LED_GRN,HIGH);
  digitalWrite(VHFPOW, LOW);
  digitalWrite(BURN, LOW);
  digitalWrite(CAM_TRIG, HIGH);
  digitalWrite(CAM_EN, HIGH);
  pinMode(2, INPUT); //Arduino Interrupt2
  pinMode(3, INPUT); //Arduino Interrupt1

  Serial.println("OT3");
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
  loadScript(); // do this early to set time
  initSensors();
  readRTC();
  Serial.print(year); Serial.print("-");
 Serial.print(month);Serial.print("-");
 Serial.print(day);Serial.print(" ");
 Serial.print(hour);Serial.print(":");
 Serial.print(minute);Serial.print(":");
 Serial.print(second);

 logFileWrite();
  
  if(burnFlag==2){
    burnTime = t + burnSeconds;
    Serial.print("Burn time set");
    Serial.println(burnTime);
  }
  if(startTime==0) startTime = t + 0; 
  Serial.print("Time:"); Serial.println(t);
  Serial.print("Start Time:"); Serial.println(startTime);
  digitalWrite(LED_GRN, LOW);
  digitalWrite(LED_RED, LOW);

  setClockPrescaler(clockprescaler); // set clockprescaler from script file

  //setupWDT(11); // initialize and activate WDT with maximum period (~500 ms)

}

void loop() {
  while(mode==0){
    // resetWdt();
    readRTC();
    checkBurn();
    Serial.print(t); Serial.print(" "); Serial.println(startTime);
    delay(1000);
    if(t >= startTime){
      endTime = startTime + recDur;
      startTime += recDur + recInt;  // this will be next start time for interval record
      fileInit();
      updateTemp();  // get first reading ready
      mode = 1;
      camStart();
      startInterruptTimer(speriod, clockprescaler);
      attachInterrupt(digitalPinToInterrupt(HALL), spinCount, RISING);
    }
  } // mode = 0


  while(mode==1){
    // resetWdt();
    readRTC();
    checkVHF();
    checkBurn();
    
    // check if time to close
    if(t>=endTime){
      stopTimer();
      dataFile.close(); // close file
      LED_EN = 0; // disable green LED flashing after first file
      if(recInt==0){  // no interval between files
        endTime += recDur;  // update end time
        fileInit();
        startInterruptTimer(speriod, clockprescaler);
      }
      else{
        mode = 0;
      }
    }

    // Check if stop button pressed
    if(digitalRead(BUTTON1)==0){
      delay(10); // simple deBounce
      if(digitalRead(BUTTON1)==0){
        camStop();
        stopTimer();
        digitalWrite(LED_RED, HIGH);
        dataFile.close();
        delay(30000);
        // wait 30 s to stop
        startInterruptTimer(speriod, clockprescaler);
        fileInit();
        digitalWrite(LED_RED, LOW);
      }
    }

    // Check if voltage is low
    if(voltage < 3.3){
      readVoltage(); // check again to be sure
      if(voltage < 3.3){
        camStop();
        stopTimer();
        dataFile.close();

        // go into low power mode
        mpuInit(0); // sleep IMU
        islSleep(); // sleep light sensor
        setClockPrescaler(3); // run at 1 MHz

        delay(1000);
        camPowOff();

        digitalWrite(VHFPOW, HIGH); // turn on VHF
        digitalWrite(BURN, HIGH);   // burn

        // check for burn and VHF
        while(1){
        }
      }
    }
  } // mode = 1
  
}


boolean ledState;
void spinCount(){
  ledState = !ledState;
  if(HALL_EN) digitalWrite(LED_RED, ledState);
  spin++;

}

void initSensors(){

//
//  Serial.print("Stop");
//  for(int x = 0; x<100; x++){
//    Serial.println(digitalRead(BUTTON1));
//    delay(100);
//  }

// Battery Voltage
  for(int x = 0; x<10; x++){
    readVoltage();
    Serial.println(voltage);
    delay(100);
  }

  reset_alarm();
//  Serial.println(rtcStatus());
//  for(int i=0; i<10; i++){
//    readRTC();
//    Serial.println(second);
//    delay(1000);
//  }
  // flash LED with current hour
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GRN, LOW);
  delay(1000);
  readRTC();
  for(int i=0; i<hour; i++){
    delay(300);
    digitalWrite(LED_GRN, HIGH);
    delay(100);
    digitalWrite(LED_GRN, LOW);
  }
  digitalWrite(LED_RED, LOW);

  // Pressure/Temperature
  pressInit();
  for(int x=0; x<10; x++){
    updatePress();
    delay(100);
    readPress();
    updateTemp();
    delay(100);
    readTemp();
    calcPressTemp();
    Serial.print(" press:"); Serial.print(pressure_mbar);
    Serial.print(" depth:"); Serial.print(depth);
    Serial.print(" temp:"); Serial.println(temperature);
  }


  islInit();
  Serial.println("RGB");
  for (int x=0; x<20; x++){
    islRead();
    Serial.print(islRed); Serial.print("\t");
    Serial.print(islGreen); Serial.print("\t");
    Serial.println(islBlue);
    delay(100);
  }
  
  mpuInit(1);
  for(int i=0; i<30; i++){
      readImu();
      calcImu();
      printImu();
      delay(100);
    }
}

void calcImu(){
      accelX = (int16_t) ((int16_t)imuTempBuffer[0] << 8 | imuTempBuffer[1]);    
      accelY = (int16_t) ((int16_t)imuTempBuffer[2] << 8 | imuTempBuffer[3]);   
      accelZ = (int16_t) ((int16_t)imuTempBuffer[4] << 8 | imuTempBuffer[5]);    
      
     // gyroTemp = (int16_t) (((int16_t)imuTempBuffer[6]) << 8 | imuTempBuffer[7]);   
     
      gyroX = (int16_t)  (((int16_t)imuTempBuffer[8] << 8) | imuTempBuffer[9]);   
      gyroY = (int16_t)  (((int16_t)imuTempBuffer[10] << 8) | imuTempBuffer[11]); 
      gyroZ = (int16_t)  (((int16_t)imuTempBuffer[12] << 8) | imuTempBuffer[13]);   
      
      magX = (int16_t)  (((int16_t)imuTempBuffer[14] << 8) | imuTempBuffer[15]);   
      magY = (int16_t)  (((int16_t)imuTempBuffer[16] << 8) | imuTempBuffer[17]);   
      magZ = (int16_t)  (((int16_t)imuTempBuffer[18] << 8) | imuTempBuffer[19]);  
}

void printImu(){
      Serial.print("a/m/g:\t");
      Serial.print(accelX); Serial.print("\t");
      Serial.print(accelY); Serial.print("\t");
      Serial.print(accelZ); Serial.print("\t");
      Serial.print(magX); Serial.print("\t");
      Serial.print(magY); Serial.print("\t");
      Serial.print(magZ); Serial.print("\t");
      Serial.print(gyroX); Serial.print("\t");
      Serial.print(gyroY); Serial.print("\t");
      Serial.println(gyroZ);
}

void fileWriteImu(){
  dataFile.print(accelX); dataFile.print(",");
  dataFile.print(accelY); dataFile.print(",");
  dataFile.print(accelZ); dataFile.print(",");
  dataFile.print(magX); dataFile.print(",");
  dataFile.print(magY); dataFile.print(",");
  dataFile.print(magZ); dataFile.print(",");
  dataFile.print(gyroX); dataFile.print(",");
  dataFile.print(gyroY); dataFile.print(",");
  dataFile.print(gyroZ);
}

void fileWriteSlowSensors(){
   dataFile.print(','); dataFile.print(year);  
    dataFile.print('-');
    if(month < 10) dataFile.print('0');
    dataFile.print(month);
    dataFile.print('-');
    if(day < 10) dataFile.print('0');
    dataFile.print(day);
    dataFile.print('T');
    if(hour) dataFile.print('0');
    dataFile.print(hour);
    dataFile.print(':');
   if(minute < 10) dataFile.print('0');
   dataFile.print(minute);
   dataFile.print(':');
   if(second < 10) dataFile.print('0');
   dataFile.print(second);
   dataFile.print("Z,");
    dataFile.print(islRed);
    dataFile.print(','); dataFile.print(islGreen);
    dataFile.print(','); dataFile.print(islBlue);
    dataFile.print(','); dataFile.print(pressure_mbar);
    dataFile.print(','); dataFile.print(depth);
    dataFile.print(','); dataFile.print(temperature);
    dataFile.print(','); dataFile.print(spin);
    dataFile.print(','); dataFile.print(voltage);
}

void logFileWrite()
{
   readRTC();
   
   File logFile = sd.open("log.txt", O_WRITE | O_CREAT | O_APPEND);
   logFile.print("Code version:"); logFile.println(codeVer);
   logFile.print("Serial Number: ");
   for (uint8_t i = 14; i < 24; i += 1) {
       logFile.print(boot_signature_byte_get(i), HEX);
   }
   logFile.println();
   logFile.print(year);  logFile.print("-");
   logFile.print(month); logFile.print("-");
   logFile.print(day); logFile.print("T");
   logFile.print(hour); logFile.print(":");
   logFile.print(minute); logFile.print(":");
   logFile.println(second);

   logFile.close();
}


void flatFileOpen()
{
   readRTC();
   
   dataFile = sd.open("flat.csv", O_WRITE | O_CREAT);
   dataFile.println("accelX,accelY,accelZ,magX,magY,magZ");
   SdFile::dateTimeCallback(file_date_time);
}

void fileInit()
{
   char filename[60];
   sprintf(filename,"%02d%02d%02dT%02d%02d%02d.csv", year, month, day, hour, minute, second);  //filename is DDHHMM
   dataFile = sd.open(filename, O_WRITE | O_CREAT | O_APPEND);
   while (!dataFile){
    fileCount += 1;
    sprintf(filename,"F%06d.amx",fileCount); //if can't open just use count
    dataFile = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
    Serial.println(filename);
    delay(100);
   }
   dataFile.println("accelX,accelY,accelZ,magX,magY,magZ,gyroX,gyroY,gyroZ,date,red,green,blue,mBar,depth,temperature,spin,V");
   SdFile::dateTimeCallback(file_date_time);
   Serial.println(filename);
}

/********************************************************************
***        Master Interrupt Routine to Read Sensors               ***
********************************************************************/
void sampleSensors(void){  
    ssCounter++;
    readImu();
    calcImu();
    fileWriteImu();

  // MS58xx start temperature conversion half-way through
  if((ssCounter>=(0.5 * slowRateMultiple))  & togglePress){ 
    readPress();   
    updateTemp();
    togglePress = 0;
  }
    
    if(ssCounter>=slowRateMultiple){
      // MS58xx pressure and temperature
      readTemp();
      updatePress();  
      togglePress = 1;
      
      if(LED_EN) digitalWrite(LED_GRN, HIGH);
      islRead(); // RGB in between to give temperature time to convert
      readRTC();
      calcPressTemp(); // MS58xx pressure and temperature
      readVoltage();
      fileWriteSlowSensors();
      ssCounter = 0;
      spin = 0; //reset spin counter
      digitalWrite(LED_GRN, LOW);
    }
    
    dataFile.println();
}

//This function returns the date and time for SD card file access and modify time. One needs to call in setup() to register this callback function: SdFile::dateTimeCallback(file_date_time);
void file_date_time(uint16_t* date, uint16_t* time) 
{
  *date=FAT_DATE(year + 2000,month,day);
  *time=FAT_TIME(hour,minute,second);
}

int checkBurn(){
  if((t>=burnTime) & (burnFlag>0)){
    digitalWrite(BURN, HIGH);
    digitalWrite(VHFPOW, HIGH);
  }
}

void checkVHF(){
  if(depth<1.0) {
      digitalWrite(VHFPOW, HIGH);
    }
    else{
      if((depth>1.5)) {
        digitalWrite(VHFPOW, LOW);
      }
    }
}

void readVoltage(){
  voltage = analogRead(BAT_VOLTAGE) * 0.0042;
}

void startInterruptTimer(int speriod, byte clockprescaler){
    MsTimer2::set(speriod>>clockprescaler, sampleSensors); // bitshift by clockprescaler...will round if not even
    MsTimer2::start();
}

void stopTimer(){
    MsTimer2::stop();
}

void camStart(){
    digitalWrite(CAM_TRIG, LOW);
    delay(500);
    digitalWrite(CAM_TRIG, HIGH);
}

void camStop(){
    digitalWrite(CAM_TRIG, LOW);
    delay(500);
    digitalWrite(CAM_TRIG, HIGH);
}

void camPowOff(){
  digitalWrite(CAM_EN, LOW);
}

void camPowOn(){
  digitalWrite(CAM_EN, HIGH);
}

