// from https://github.com/ROHMUSDC/ROHM_SensorPlatform_Multi-Sensor-Shield/blob/master/Platform%20Code/Arduino_UNO_FirmwareExample/ROHM_SENSORSHLD1-EVK-101_10-20-2016/ROHM_SENSORSHLD1-EVK-101_TerminalDemo_11-10-2016/ROHM_SENSORSHLD1-EVK-101_TerminalDemo_11-10-2016.ino

int KMX62_DeviceAddress = 0x0E; 
#define KMX_WHO_AM_I (0x00)
#define KMX_INS1 (0x01)
#define KMX_INS2 (0x02)
#define KMX_INS3 (0x03)
#define KMX_INL (0x04)
#define KMX_ACCEL (0x0A)
#define KMX_MAG (0x10)
#define KMX_TEMP (0x16)
#define KMX_INC1 (0x2A)
#define KMX_INC2 (0x2B)
#define KMX_INC3 (0x2C)
#define KMX_INC4 (0x2D)
#define KMX_INC5 (0x2E)
#define KMX_AMI_CNTL1 (0x2F)
#define KMX_AMI_CNTL2 (0x30)
#define KMX_AMI_CNTL3 (0x31)
#define KMX_MMI_CNTL1 (0x32)
#define KMX_MMI_CNTL2 (0x33)
#define KMX_MMI_CNTL3 (0x34)
#define KMX_FFI_CNTL1 (0x35)
#define KMX_FFI_CNTL2 (0x36)
#define KMX_FFI_CNTL3 (0x37)
#define KMX_ODCNTL (0x38)
#define KMX_CNTL1 (0x39)
#define KMX_CNTL2 (0x3A)
#define KMX_COTR (0x3C)
#define BUF_CTRL_1 (0x77)
#define BUF_CTRL_2 (0x78)
#define BUF_CTRL_3 (0x79)
#define BUF_CLEAR (0x7A)
#define BUF_STATUS_1 (0x7B)
#define BUF_STATUS_2 (0x7C)
#define BUF_STATUS_3 (0x7D)
#define BUF_READ (0x7E)

//Accel Portion
int MEMS_Accel_Xout_highByte = 0;
int MEMS_Accel_Xout_lowByte = 0;
int MEMS_Accel_Yout_highByte = 0;
int MEMS_Accel_Yout_lowByte = 0;
int MEMS_Accel_Zout_highByte = 0;
int MEMS_Accel_Zout_lowByte = 0;

//Mag Sensor Portion
int MEMS_Mag_Xout_highByte = 0;
int MEMS_Mag_Xout_lowByte = 0;
int MEMS_Mag_Yout_highByte = 0;
int MEMS_Mag_Yout_lowByte = 0;
int MEMS_Mag_Zout_highByte = 0;
int MEMS_Mag_Zout_lowByte = 0;


 //----- Start Initialization for KMX62 Digital Accel/Mag Sensor -----
 //KMX62 Init Sequence
  // 1. Standby Register (0x29), write 0x03 (Turn Off)
  // 2. Self Test Register (0x60), write 0x00
  // 3. Control Register 1 (0x2A), write 0x13
  // 4. Control Register 2 (0x2B), write 0x00
  // 5. ODCNTL Register (0x2C), write 0x00
  // 6. Temp EN Register (0x4C), write 0x01
  // 7. Buffer CTRL Register 1 (0x78), write 0x00
  // 8. Buffer CTRL Register 2 (0x79), write 0x00
  // 9. Standby Register (0x29), write 0x0u

  //KMX62 Init Sequence
  // 1. CNTL2 (0x3A), write (0x5F): 4g, Max RES, EN temp mag and accel

void writeI2C(int devAddress, int registerAddress, int value){
  Wire.beginTransmission(devAddress);
  Wire.write(registerAddress);
  Wire.write(value);
  Wire.endTransmission();
}

int kmx62Init(){

  

  // enable accelerometer and magnetometer
  writeI2C(KMX62_DeviceAddress, KMX_CNTL2, 0x5F);
  delay(2);

  return 1;
}

int kmx62TestResponse(){  // should return 0x55
  Wire.beginTransmission(KMX62_DeviceAddress);
  Wire.write(KMX_COTR);
  Wire.endTransmission();
  
  Wire.requestFrom(KMX62_DeviceAddress, 1);
  return(Wire.read());
  Wire.endTransmission();
}

 //---------- Start Code for Reading KMX62 Kionix Accelerometer+Mag Sensor ----------  
  // -- Notes on ROHM KMX62 Accel/Mag Sensor --
  //Device Address = 0x0Eu
  //12 Bit Return Value
  
  //Intialization Routines (See Setup function above)
  // 1. Standby Register (0x29), write 0x03 (Turn Off)
  // 2. Self Test Register (0x60), write 0x00
  // 3. Control Register 1 (0x2A), write 0x13
  // 4. Control Register 2 (0x2B), write 0x00
  // 5. ODCNTL Register (0x2C), write 0x00
  // 6. Temp EN Register (0x4C), write 0x01
  // 7. Buffer CTRL Register 1 (0x78), write 0x00
  // 8. Buffer CTRL Register 2 (0x79), write 0x00
  // 9. Standby Register (0x29), write 0x0u
  
  //Main Loop Routines
  // 1. Read 6 Bytes starting from address 0x0A.  These will be the accelerometer output. [0][1]...[5]
  // 2. Xout = ([1]<<6) | ([0]>>2)
  // 3. Yout = ([3]<<6) | ([2]>>2)
  // 4. Zout = ([5]<<6) | ([4]>>2)
  // 5. Read 6 Bytes starting from addres 0x12. These will be the magnetometer output. [0][1]...[5]  
  // 6. Xout = ([1]<<6) | ([0]>>2)
  // 7. Yout = ([3]<<6) | ([2]>>2)
  // 8. Zout = ([5]<<6) | ([4]>>2)

  void readKMX62(){
  // Start Getting Data from Accel
  Wire.beginTransmission(KMX62_DeviceAddress);
  Wire.write(0x0A);
  Wire.endTransmission(0);
  
  Wire.requestFrom(KMX62_DeviceAddress, 6, 0);
  //Serial.println(Wire.available());
  MEMS_Accel_Xout_lowByte = Wire.read();
  MEMS_Accel_Xout_highByte = Wire.read();
  MEMS_Accel_Yout_lowByte = Wire.read();
  MEMS_Accel_Yout_highByte = Wire.read();
  MEMS_Accel_Zout_lowByte = Wire.read();
  MEMS_Accel_Zout_highByte = Wire.read();
  Wire.endTransmission();
  
  accelX = (MEMS_Accel_Xout_highByte<<8) | (MEMS_Accel_Xout_lowByte);
  accelY = (MEMS_Accel_Yout_highByte<<8) | (MEMS_Accel_Yout_lowByte);
  accelZ = (MEMS_Accel_Zout_highByte<<8) | (MEMS_Accel_Zout_lowByte);
  
  Wire.beginTransmission(KMX62_DeviceAddress);
  Wire.write(0x10);
  Wire.endTransmission(0);
  Wire.requestFrom(KMX62_DeviceAddress, 6, 0);
  MEMS_Mag_Xout_lowByte = Wire.read();
  MEMS_Mag_Xout_highByte = Wire.read();
  MEMS_Mag_Yout_lowByte = Wire.read();
  MEMS_Mag_Yout_highByte = Wire.read();
  MEMS_Mag_Zout_lowByte = Wire.read();
  MEMS_Mag_Zout_highByte = Wire.read();
  Wire.endTransmission();
  
  magX = (MEMS_Mag_Xout_highByte<<8) | (MEMS_Mag_Xout_lowByte);
  magY = (MEMS_Mag_Yout_highByte<<8) | (MEMS_Mag_Yout_lowByte);
  magZ = (MEMS_Mag_Zout_highByte<<8) | (MEMS_Mag_Zout_lowByte);
  }  
