// from https://github.com/ROHMUSDC/ROHM_SensorPlatform_Multi-Sensor-Shield/blob/master/Platform%20Code/Arduino_UNO_FirmwareExample/ROHM_SENSORSHLD1-EVK-101_10-20-2016/ROHM_SENSORSHLD1-EVK-101_TerminalDemo_11-10-2016/ROHM_SENSORSHLD1-EVK-101_TerminalDemo_11-10-2016.ino

int kmx62Address = 0x1C; //this is 8 bit address, 7-bit address = 0x0E
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
#define KMX_BUF_CTRL_1 (0x77)
#define KMX_BUF_CTRL_2 (0x78)
#define KMX_BUF_CTRL_3 (0x79)
#define KMX_BUF_CLEAR (0x7A)
#define KMX_BUF_STATUS_1 (0x7B)
#define KMX_BUF_STATUS_2 (0x7C)
#define KMX_BUF_STATUS_3 (0x7D)
#define KMX_BUF_READ (0x7E)

#define KMX_25HZ (0x01)
#define KMX_50HZ (0x02)
#define KMX_100HZ (0x03)
#define KMX_200HZ (0x04)
#define KMX_400HZ (0x05)
#define KMX_800HZ (0x06)
#define KMX_1600HZ (0x07)
#define KMX_25KHZ (0xFF)

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
  i2c_start(devAddress); //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(registerAddress);
  i2c_write(value);
  i2c_stop();
  __asm__ __volatile__ ("nop\n\t"); // very short delay
}

void kmx62Reset(){
  writeI2C(kmx62Address, KMX_CNTL1, 0x80);
  delay(5);
}

int kmx62Init(int fifoMode){
  writeI2C(kmx62Address, KMX_INC1, 0x20); // FIFO watermark reported on GPIO1
  writeI2C(kmx62Address, KMX_INC2, 0x01); // Magnetometer motion interrupt reported on GPIO2
  writeI2C(kmx62Address, KMX_INC3, 0x45); // push-pull, active high, GPIO1 pulsed, GPIO2 latched
  writeI2C(kmx62Address, KMX_INC5, 0x03); // magnetometer Z axis for motion interrupt
  writeI2C(kmx62Address, KMX_MMI_CNTL1, 0xFF); // magnetometer interrupt threshold is max value
  writeI2C(kmx62Address, KMX_MMI_CNTL2, 0xFF); // magnetometer motion counter 255
  writeI2C(kmx62Address, KMX_MMI_CNTL3, 0xC7); // magentometer motion interrupt enabled, unlatched, ODR at 100 Hz

  if(fifoMode) { 
    writeI2C(kmx62Address, KMX_BUF_CTRL_1, 0xC0); // watermark at 192 samples
    writeI2C(kmx62Address, KMX_BUF_CTRL_2, 0x00); // FIFO mode
    writeI2C(kmx62Address, KMX_BUF_CTRL_3, 0xFE); // buffer full interrupt enabled, accel and mag buffered, temp not buffered
  }
  return 1;
}

void kmx62SampleRate(int srate){
    writeI2C(kmx62Address, KMX_CNTL2, 0x00); // standby mode
    delay(1);
    switch(srate){
      case 25: 
        writeI2C(kmx62Address, KMX_ODCNTL, (KMX_25HZ<<4) | KMX_25HZ); //Mag and Accel update rate
        break;
      case 50: 
        writeI2C(kmx62Address, KMX_ODCNTL, (KMX_50HZ<<4) | KMX_50HZ); //Mag and Accel update rate
        break;
      case 100: 
        writeI2C(kmx62Address, KMX_ODCNTL, (KMX_100HZ<<4) | KMX_100HZ); //Mag and Accel update rate
        break;
      case 200: 
        writeI2C(kmx62Address, KMX_ODCNTL, (KMX_200HZ<<4) | KMX_200HZ); //Mag and Accel update rate
        break;
      case 400: 
        writeI2C(kmx62Address, KMX_ODCNTL, (KMX_400HZ<<4) | KMX_400HZ); //Mag and Accel update rate
        break;
      case 800: 
        writeI2C(kmx62Address, KMX_ODCNTL, (KMX_800HZ<<4) | KMX_800HZ); //Mag and Accel update rate
        break;
      case 1600: 
        writeI2C(kmx62Address, KMX_ODCNTL, (KMX_1600HZ<<4) | KMX_1600HZ); //Mag and Accel update rate
        break;
      case 25000: 
        writeI2C(kmx62Address, KMX_ODCNTL, KMX_25KHZ); // Accelerometer ONLY
        break;
    }
}

void kmx62ClearFifo(){
  writeI2C(kmx62Address, KMX_BUF_CLEAR, 0x01);
}

int kmx62GetFifoPoints(){
  byte nFifoPts_low, nFifoPts_high;
  i2c_start(kmx62Address);
  i2c_write(KMX_BUF_STATUS_1);
  i2c_rep_start(kmx62Address | 1); // or'd with 1 for read bit
  nFifoPts_low = i2c_read(false);
  nFifoPts_high = i2c_read(true) & 0x01; //only need bit 0
  i2c_stop();
  return (nFifoPts_high<<8 | nFifoPts_low);
}
void kmx62Start(unsigned int setting){
  // 0x5F
  // temperature standby
  // +/- 16g accelerometer range
  // mag and accel maximum oversample
  // enable accelerometer and magnetometer
  writeI2C(kmx62Address, KMX_CNTL2, setting); 
}

int kmx62TestResponse(){  // should return 0x55 (decimal 85)
  byte response;
  i2c_start(kmx62Address);
  i2c_write(KMX_COTR);
  
  i2c_rep_start(kmx62Address | 1); // or'd with 1 for read bit
  response = i2c_read(true);
  i2c_stop();
  return response;
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

  void kmx62Read(){
    // Start Getting Data from Accel
    i2c_start(kmx62Address);
    i2c_write(0x0A);
    i2c_rep_start(kmx62Address | 1);  // Or-ed with "1" for read bit
    MEMS_Accel_Xout_lowByte = i2c_read(false);
    MEMS_Accel_Xout_highByte = i2c_read(false);
    MEMS_Accel_Yout_lowByte = i2c_read(false);
    MEMS_Accel_Yout_highByte = i2c_read(false);
    MEMS_Accel_Zout_lowByte = i2c_read(false);
    MEMS_Accel_Zout_highByte = i2c_read(true);
    i2c_stop();
    
    accelX = (MEMS_Accel_Xout_highByte<<8) | (MEMS_Accel_Xout_lowByte);
    accelY = (MEMS_Accel_Yout_highByte<<8) | (MEMS_Accel_Yout_lowByte);
    accelZ = (MEMS_Accel_Zout_highByte<<8) | (MEMS_Accel_Zout_lowByte);
    
    // Start Getting Data from Mag Sensor
    i2c_start(kmx62Address);
    i2c_write(0x10);
    i2c_rep_start(kmx62Address | 1);  // Or-ed with "1" for read bit
    MEMS_Mag_Xout_lowByte = i2c_read(false);
    MEMS_Mag_Xout_highByte = i2c_read(false);
    MEMS_Mag_Yout_lowByte = i2c_read(false);
    MEMS_Mag_Yout_highByte = i2c_read(false);
    MEMS_Mag_Zout_lowByte = i2c_read(false);
    MEMS_Mag_Zout_highByte = i2c_read(true);
    i2c_stop();
    
    magX = (MEMS_Mag_Xout_highByte<<8) | (MEMS_Mag_Xout_lowByte);
    magY = (MEMS_Mag_Yout_highByte<<8) | (MEMS_Mag_Yout_lowByte);
    magZ = (MEMS_Mag_Zout_highByte<<8) | (MEMS_Mag_Zout_lowByte);
  }  

  void kmx62FifoRead(){  
    i2c_start(kmx62Address);
    i2c_write(KMX_BUF_READ);
    i2c_rep_start(kmx62Address | 1);
    for(int n=0; n<BUF_BYTES-1; n++){
      fifoVal[n] = i2c_read(false);
    }
    fifoVal[BUF_BYTES-1] = i2c_read(true);
    i2c_stop();
  }

