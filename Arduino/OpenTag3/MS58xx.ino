// TE 58xx Pressure Sensor

#define POW2_6 64
#define POW2_7 128
#define POW2_8 256
#define POW2_15 32768
#define POW2_16 65536
#define POW2_17 131072
#define POW2_21 2097152
#define POW2_23 8388608


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
  
  float dT = (float) D2 - ((float) TREF * POW2_8);
  float T16 = 2000.0 + (dT * (float) TEMPSENS / (float) POW2_23);
  temperature = T16 / 100.0;

  float OFF, SENS;
  if(MS58xx_constant==327680.0){
    OFF = ((float) POFF * POW2_17)  + (((float) TCOFF * dT) / POW2_6);
    SENS = ((float) PSENS * POW2_16) + ((dT * (float) TCSENS) / POW2_7);
  }
  else{
    OFF = ((float) POFF * POW2_16)  + (((float) TCOFF * dT) / POW2_7);
    SENS = ((float) PSENS * POW2_15) + ((dT * (float) TCSENS) / POW2_8);
  }
    
  pressure_mbar = (((float) D1 * SENS / POW2_21 ) - OFF) / MS58xx_constant / 10.0;  // mbar
  float mbar_per_m = 111.377;
  depth = -(1010.0 -  pressure_mbar) / mbar_per_m;
  temperature = T16 / 100.0;
}

