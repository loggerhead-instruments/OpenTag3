// TE 58xx Pressure Sensor


int pressInit()
{
  int i = 0;
  byte buff[2];
  int bytesread;

  if (printDiags) Serial.println("MS58xx Init");
  // Reset so PROM is loaded
  Wire.beginTransmission(pressAddress);
  Wire.write(0x1E);  //Reset Command
  bytesread = Wire.endTransmission();
  delay(5);  //reset needs at least 2.8 ms
  
  // Read and store calibration coefficients
  Wire.beginTransmission(pressAddress);
  Wire.write(0xA2);  //PROM Read Pressure Sensitivity
  Wire.endTransmission();
  
  bytesread = Wire.requestFrom(pressAddress, 2);    // request 2 bytes from device
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  PSENS = ((uint16_t) buff[0]<<8)| (uint16_t) buff[1]; //pressure sensitivity  MSB first
    
  i=0;
  Wire.beginTransmission(pressAddress);
  Wire.write(0xA4);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  POFF = ((uint16_t) buff[0]<<8) | (uint16_t) buff[1];  //Pressure offset
 
  i=0;
  Wire.beginTransmission(pressAddress);
  Wire.write(0xA6);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  TCSENS = ((uint16_t)  buff[0] << 8) | (uint16_t) buff[1];  //

  i=0;
  Wire.beginTransmission(pressAddress);
  Wire.write(0xA8);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }

  TCOFF = ((uint16_t) buff[0]<<8) | (uint16_t) buff[1];  //Temp coefficient of pressure offset
  
  i=0;
  Wire.beginTransmission(pressAddress);
  Wire.write(0xAA);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }

  TREF = ((uint16_t) buff[0]<<8) | (uint16_t) buff[1];  //Ref temperature
  
  i=0;
  Wire.beginTransmission(pressAddress);
  Wire.write(0xAC);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }

  TEMPSENS = ((uint16_t) buff[0]<<8) | (uint16_t) buff[1];  //Temperature sensitivity coefficient  
  return (i>0);  // return 1 if value returned for calibration coefficient; otherwise 0
}

void updatePress()
{
    Wire.beginTransmission(pressAddress);
    Wire.write(0x48);  //Initiate pressure conversion OSR-4096
    Wire.endTransmission();

}

void updateTemp()
{
   Wire.beginTransmission(pressAddress);
   Wire.write(0x58); //Initiate Temperature conversion OSR=4096
   Wire.endTransmission();
}

void readPress()
{
  int i = 0;
  
  Wire.beginTransmission(pressAddress);
  Wire.write((byte) 0);  //ADC Read Command
  Wire.endTransmission();
  
  Wire.requestFrom(pressAddress, 3);    // request 3 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    Pbuff[i] = Wire.read();  // receive one byte
    i++;
  }
}

void readTemp()
{
  int i = 0;
 
  Wire.beginTransmission(pressAddress);
  Wire.write((byte) 0);  //ADC Read Command
  Wire.endTransmission();
  
  Wire.requestFrom(pressAddress, 3);    // request 3 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    Tbuff[i] = Wire.read();  // receive one byte
    i++;
  }
}

// PSENS; //pressure sensitivity C1
// POFF;  //Pressure offset C2
// TCSENS; //Temp coefficient of pressure sensitivity C3
// TCOFF; //Temp coefficient of pressure offset C4
// TREF;  //Ref temperature C5
// TEMPSENS; //Temperature sensitivity coefficient C6
void calcPressTemp(){
  uint32_t D1 = (uint32_t)((((uint32_t)Pbuff[0]<<16) | ((uint32_t)Pbuff[1]<<8) | ((uint32_t) Pbuff[2])));
  uint32_t D2 = (uint32_t)((((uint32_t)Tbuff[0]<<16) | ((uint32_t)Tbuff[1]<<8) | ((uint32_t) Tbuff[2])));
  
  float dT = (float) D2 - ((float) TREF * 256.0);
  float T16 = 2000.0 + (dT * (float) TEMPSENS / (float) 8388608.0);

  float OFF, SENS;
  #ifdef MS58xx_05bar
    OFF = ((float) POFF * 262144.0)  + (((float) TCOFF * dT) / 32.0);
    SENS = ((float) PSENS * 131072.0) + ((dT * (float) TCSENS) / 128.0);
  #else
    OFF = ((float) POFF * 65536.0)  + (((float) TCOFF * dT) / 128.0);
    SENS = ((float) PSENS * 32768.0) + ((dT * (float) TCSENS) / 256.0);
  #endif
    
  pressure_mbar = ((float) D1 * SENS / 2097152.0 - OFF) / MS58xx_constant / 10.0;  // mbar
  float mbar_per_m = 1113.77;
  depth = -(1010.0 -  pressure_mbar) / mbar_per_m;
  temperature = T16 / 100.0;
}

