/* EM7180_MPU9250_BMP280_M24512DFC Basic Example Code
 by: Kris Winer
 date: September 11, 2015
 
 adapted by: Robert Huitema
 date: Marcg 26, 2017
 
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy us a beer some time.
 
 The EM7180 SENtral sensor hub is not a motion sensor, but rather takes raw sensor data from a variety of motion sensors,
 in this case the MPU9250 (with embedded MPU9250 + AK8963C), and does sensor fusion with quaternions as its output. The SENtral loads firmware from the
 on-board M24512DRC 512 kbit EEPROM upon startup, configures and manages the sensors on its dedicated master I2C bus,
 and outputs scaled sensor data (accelerations, rotation rates, and magnetic fields) as well as quaternions and
 heading/pitch/roll, if selected.
 
 This sketch demonstrates basic EM7180 SENtral functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms to compare with the hardware sensor fusion results.
 Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 This sketch is specifically for the  Butterfly STM32L433CC development board using the EM7180 SENtral sensor hub as master,
 the MPU9250 9-axis motion sensor (accel/gyro/mag) as slave, a BMP280 pressure/temperature sensor, and an M24512DRC
 512kbit (64 kByte) EEPROM as slave all connected via I2C. The SENtral can use the pressure data in the sensor fusion
 yet and there is a driver for the BMP280 in the SENtral firmware. 
 
 This sketch uses SDA/SCL on pins 20/12, respectively, and it uses the Arduino Wire library.
 The BMP280 is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.
 
 4k7 resistors are on the EM7180+MPU9250+BMP280+M24512DRC Mini Add-On board for Butterfly STM32L433CC.
 
 Hardware setup:
 EM7180 IMU -------    Butterfly STM32L433CC
 VDD ---------------------- 3.3V
 SDA ----------------------- 20
 SCL ----------------------- 21
 GND ---------------------- GND
 INT------------------------ 7
 
 */
 
#include "Arduino.h"
#include "Globals.h"

#include "Wire.h"  
#include <SPI.h>

#define greenLed 38 // green led
#define redLed 13 // red led
#define blueLed 26 // blue led 

#define SerialDebug true
//#define SerialDebug false

bool passThru = false;

//float VDDA, Temperature;
void setup()
{  
  pinMode(PIN_BUTTON, INPUT);
  pinMode(blueLed, OUTPUT);
  digitalWrite(blueLed, LOW);  // start with led on, since active LOW
 
  delay(100); 
  
  // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
 // Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  Wire.begin(TWI_PINS_20_21); // set master mode on pins 21/20
  Wire.setClock(400000); // I2C frequency at 400 kHz
  delay(100);
  digitalWrite(blueLed, !digitalRead(blueLed)); // toggle blue led off
  // Read Acc offset info
  //eeprom->readGlobalSet();
  delay(100);
  Serial.begin(115200);
  delay(5000);
  if(SerialDebug)Serial.println("Starting boot..");
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(redLed, OUTPUT);
  digitalWrite(redLed, LOW);

  // should detect SENtral at 0x28
  //I2Cscan();
  
  // Read SENtral device information
  uint16_t ROM1 = readByte(EM7180_ADDRESS, EM7180_ROMVersion1);
  uint16_t ROM2 = readByte(EM7180_ADDRESS, EM7180_ROMVersion2);
  if(SerialDebug){
  	Serial.print("EM7180 ROM Version: 0x"); Serial.print(ROM1, HEX); Serial.println(ROM2, HEX); Serial.println("Should be: 0xE609");
  }
  uint16_t RAM1 = readByte(EM7180_ADDRESS, EM7180_RAMVersion1);
  uint16_t RAM2 = readByte(EM7180_ADDRESS, EM7180_RAMVersion2);
  if(SerialDebug){
  	Serial.print("EM7180 RAM Version: 0x"); Serial.print(RAM1); Serial.println(RAM2);
  	}
  uint8_t PID = readByte(EM7180_ADDRESS, EM7180_ProductID);
  if(SerialDebug){
  	Serial.print("EM7180 ProductID: 0x"); Serial.print(PID, HEX); Serial.println(" Should be: 0x80");
  	}
  uint8_t RID = readByte(EM7180_ADDRESS, EM7180_RevisionID);
  if(SerialDebug){
  	Serial.print("EM7180 RevisionID: 0x"); Serial.print(RID, HEX); Serial.println(" Should be: 0x02");
  	}
  digitalWrite(redLed, !digitalRead(blueLed)); // toggle red led off
  // Give some time to read the screen
  delay(1000);

  // Check which sensors can be detected by the EM7180
  uint8_t featureflag = readByte(EM7180_ADDRESS, EM7180_FeatureFlags);
  if(SerialDebug){
		if(featureflag & 0x01)  Serial.println("A barometer is installed");
		if(featureflag & 0x02)  Serial.println("A humidity sensor is installed");
		if(featureflag & 0x04)  Serial.println("A temperature sensor is installed");
		if(featureflag & 0x08)  Serial.println("A custom sensor is installed");
		if(featureflag & 0x10)  Serial.println("A second custom sensor is installed");
		if(featureflag & 0x20)  Serial.println("A third custom sensor is installed");
  }
  // Give some time to read the screen
  delay(1000);

  // Check SENtral status, make sure EEPROM upload of firmware was accomplished
  byte STAT = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
   if(SerialDebug){
		if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("EEPROM detected on the sensor bus!");
		if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("EEPROM uploaded config file!");
		if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("EEPROM CRC incorrect!");
		if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("EM7180 in initialized state!");
		if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("No EEPROM detected!");
		}
  int count = 0;
  while(!STAT)
  {
    writeByte(EM7180_ADDRESS, EM7180_ResetRequest, 0x01);
    delay(500);  
    count++;  
    STAT = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
    if(SerialDebug){
			if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("EEPROM detected on the sensor bus!");
			if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("EEPROM uploaded config file!");
			if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("EEPROM CRC incorrect!");
			if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("EM7180 in initialized state!");
			if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("No EEPROM detected!");
			}
    if(count > 10) break;
  }
  if(!(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04))   if(SerialDebug)Serial.println("EEPROM upload successful!");
  
  	// Put the Sentral in pass-thru mode
    WS_PassThroughMode();
    warm_start = checkSenParams();
    WS_Resume();
    delay(100);
    
    if(warm_start)
    {
      
      // Put the Sentral in pass-thru mode
      WS_PassThroughMode();

      // Fetch the WarmStart data from the M24512DFM I2C EEPROM
       if(SerialDebug)Serial.println("Loading warm start ...");

      readSenParams();
       if(SerialDebug)Serial.println("!!!Warm Start active!!!");


      // Take Sentral out of pass-thru mode and re-start algorithm
      WS_Resume();
    } else
    {
      if(SerialDebug)Serial.println("***No Warm Start***");
    }

    // Reset Sentral after 

  // Give some time to read the screen
  delay(500);
 
}

void loop()
{  
  if(!passThru)
  {

    serial_input = Serial.read();
   
    if ( digitalRead(PIN_BUTTON) == HIGH )
    {
      if(SerialDebug)Serial.print(" Updating warm start data... ");
      
      delay(1000);
      EM7180_get_WS_params();
      
      // Put the Sentral in pass-thru mode
      WS_PassThroughMode();

      // Store WarmStart data to the M24512DFM I2C EEPROM
      writeSenParams();

      // Take Sentral out of pass-thru mode and re-start algorithm
      WS_Resume();
      warm_start_saved = 1;
      if(SerialDebug)Serial.println(" Warm start data updated ");
    }

    // Check event status register, way to chech data ready by polling rather than interrupt
    uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register
  
    // Check for errors
    // Error detected, what is it?
    if(eventStatus & 0x02)
    { 
      uint8_t errorStatus = readByte(EM7180_ADDRESS, EM7180_ErrorRegister);
      if(!errorStatus && SerialDebug)
      {
        Serial.print(" EM7180 sensor status = "); Serial.println(errorStatus);
        if(errorStatus == 0x11) Serial.print("Magnetometer failure!");
        if(errorStatus == 0x12) Serial.print("Accelerometer failure!");
        if(errorStatus == 0x14) Serial.print("Gyro failure!");
        if(errorStatus == 0x21) Serial.print("Magnetometer initialization failure!");
        if(errorStatus == 0x22) Serial.print("Accelerometer initialization failure!");
        if(errorStatus == 0x24) Serial.print("Gyro initialization failure!");
        if(errorStatus == 0x30) Serial.print("Math error!");
        if(errorStatus == 0x80) Serial.print("Invalid sample rate!");
      }
      // Handle errors ToDo
    }
    // if no errors, see if new data is ready
    // new acceleration data available
    if(eventStatus & 0x10)
    { 
      readSENtralAccelData(accelCount);
  
      // Now we'll calculate the accleration value into actual g's
      ax = (float)accelCount[0]*0.000488;  // get actual g value
      ay = (float)accelCount[1]*0.000488;    
      az = (float)accelCount[2]*0.000488;  
    }
  
    if(eventStatus & 0x20)
    {
      // new gyro data available  
      readSENtralGyroData(gyroCount);
  
      // Now we'll calculate the gyro value into actual dps's
      gx = (float)gyroCount[0]*0.153;  // get actual dps value
      gy = (float)gyroCount[1]*0.153;    
      gz = (float)gyroCount[2]*0.153;  
    }
    
    if(eventStatus & 0x08)
    {
      // new mag data available
      readSENtralMagData(magCount);
  
      // Now we'll calculate the mag value into actual G's
      // get actual G value
      mx = (float)magCount[0]*0.305176;
      my = (float)magCount[1]*0.305176;    
      mz = (float)magCount[2]*0.305176;  
    }

    if(eventStatus & 0x04)  // new quaternions available
    {
    readSENtralQuatData(Quat); 
    }

    // get BMP280 pressure
    // new baro data available
    
    if(eventStatus & 0x40)
    { 
      rawPressure = readSENtralBaroData();
      pressure = (float)rawPressure*0.01f +1013.25f; // pressure in mBar

      // get BMP280 temperature
      rawTemperature = readSENtralTempData();  
      temperature = (float) rawTemperature*0.01;  // temperature in degrees C
    }
  }
 
  // keep track of rates
  Now = micros();

  // set integration time by time elapsed since last filter update
  deltat = ((Now - lastUpdate)/1000000.0f);
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
  // Sensors x (y)-axis of the accelerometer is aligned with the -y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ up) is aligned with z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  // For the BMX-055, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the MPU9250 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
    //MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  mx,  my, mz);
//  if(passThru)MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -my, mx, mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;

    // update LCD once per half-second independent of read rate
    if (delt_t > 500)
    { 

      if(SerialDebug)
      {
        Serial.print("ax = "); Serial.print((int)1000*ax);  
        Serial.print(" ay = "); Serial.print((int)1000*ay); 
        Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
        Serial.print("gx = "); Serial.print( gx, 2); 
        Serial.print(" gy = "); Serial.print( gy, 2); 
        Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
        Serial.print("mx = "); Serial.print( (int)mx); 
        Serial.print(" my = "); Serial.print( (int)my); 
        Serial.print(" mz = "); Serial.print( (int)mz); Serial.println(" mG");
        //Serial.println("Software quaternions (ENU):"); 
        //Serial.print("q0 = "); Serial.print(q[0]);
        //Serial.print(" qx = "); Serial.print(q[1]); 
        //Serial.print(" qy = "); Serial.print(q[2]); 
        //Serial.print(" qz = "); Serial.println(q[3]); 
        Serial.println("Hardware quaternions (NED):"); 
        Serial.print("Q0 = "); Serial.print(Quat[0]);
        Serial.print(" Qx = "); Serial.print(Quat[1]); 
        Serial.print(" Qy = "); Serial.print(Quat[2]); 
        Serial.print(" Qz = "); Serial.println(Quat[3]); 
      }               
      //if(passThru)
     // {
      //  rawPress =  readBMP280Pressure();
      //  pressure = (float) bmp280_compensate_P(rawPress)/25600.; // Pressure in mbar
      //  rawTemp =   readBMP280Temperature();
      //  temperature = (float) bmp280_compensate_T(rawTemp)/100.;
      //}
      // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
      // In this coordinate system, the positive z-axis is down toward Earth. 
      // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
      // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
      // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
      // applied in the correct order which for this configuration is yaw, pitch, and then roll.
      // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
      
      //Software AHRS:
      //yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
      //pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      //roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      //pitch *= 180.0f / PI;
      //yaw   *= 180.0f / PI; 
      //yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
      //if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
      //roll  *= 180.0f / PI;
      
      //Hardware AHRS:
      Yaw   = atan2(2.0f * (Quat[0] * Quat[1] + Quat[3] * Quat[2]), Quat[3] * Quat[3] + Quat[0] * Quat[0] - Quat[1] * Quat[1] - Quat[2] * Quat[2]);   
      Pitch = -asin(2.0f * (Quat[0] * Quat[2] - Quat[3] * Quat[1]));
      Roll  = atan2(2.0f * (Quat[3] * Quat[0] + Quat[1] * Quat[2]), Quat[3] * Quat[3] - Quat[0] * Quat[0] - Quat[1] * Quat[1] + Quat[2] * Quat[2]);
    //  Pitch *= 180.0f / PI;
     // Yaw   *= 180.0f / PI; 
      //Yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
     // if(Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
     // Roll  *= 180.0f / PI;
     
      // Or define output variable according to the Android system, where heading (0 to 360) is defined by the angle between the y-axis 
      // and True North, pitch is rotation about the x-axis (-180 to +180), and roll is rotation about the y-axis (-90 to +90)
      // In this systen, the z-axis is pointing away from Earth, the +y-axis is at the "top" of the device (cellphone) and the +x-axis
      // points toward the right of the device.
    
      if(SerialDebug)
      {
        //Serial.print("Software yaw, pitch, roll: ");
        //Serial.print(yaw, 2);
        //Serial.print(", ");
        //Serial.print(pitch, 2);
        //Serial.print(", ");
        //Serial.println(roll, 2);
        Serial.print("Hardware Yaw, Pitch, Roll: ");
        Serial.print(Yaw * 180.0f / PI, 2);
        Serial.print(", ");
        Serial.print(Pitch  * 180.0f / PI, 2);
        Serial.print(", ");
        Serial.println(Roll  * 180.0f / PI, 2);
        Serial.println("BMP280:");
        Serial.print("Altimeter temperature = "); 
        Serial.print( temperature, 2); 
        Serial.println(" C"); // temperature in degrees Celsius
        Serial.print("Altimeter temperature = "); 
        Serial.print(9.*temperature/5. + 32., 2); 
        Serial.println(" F"); // temperature in degrees Fahrenheit
        Serial.print("Altimeter pressure = "); 
        Serial.print(pressure, 2);  
        Serial.println(" mbar");// pressure in millibar
        altitude = 145366.45f*(1.0f - pow((pressure/1013.25f), 0.190284f));
        Serial.print("Altitude = "); 
        Serial.print(altitude, 2); 
        Serial.println(" feet");
        Serial.println(" ");
        if(warm_start_saved)
        {
          Serial.println("Warm Start configuration saved!");
        } else
        {
          Serial.println("Click Calibrate to store Warm Start configuration");
        }
      }
      if(SerialDebug){
      	Serial.print(millis()/1000.0, 1);Serial.print(",");
      	//Serial.print(yaw); Serial.print(",");Serial.print(pitch); Serial.print(",");Serial.print(roll); Serial.print(",");
      	Serial.print(Yaw  * 180.0f / PI); Serial.print(",");Serial.print(Pitch  * 180.0f / PI); Serial.print(",");Serial.println(Roll * 180.0f / PI);
      	}
      //output Signalk
    //heading
    Serial.print("{\"context\": \"vessels.self\",\"updates\": [{\"values\": [");
    Serial.print("{\"path\": \"navigation.headingMagnetic\",\"value\":");
    Serial.print(Yaw);
    Serial.print("},");
    //rot
    Serial.print("{\"path\": \"navigation.rateOfTurn\",\"value\":");
    Serial.print(gy*PI/180.0f);
    Serial.print("},");
    //attitude
    Serial.print("{\"path\": \"navigation.navigation.attitude\",\"value\": {");
    Serial.print("\"roll\":");
    Serial.print(Roll);
    Serial.print(",\"pitch\":");
    Serial.print(Pitch);
    Serial.print(",\"yaw\":");
    Serial.print(Yaw);
    Serial.print("}},");                
    //temp
    Serial.print("{\"path\": \"environment.inside.temperature\",\"value\":");
    Serial.print(temperature-273.15f, 2);
    Serial.print("},");
    //pressure
    Serial.print("{\"path\": \"environment.outside.pressure\",\"value\":");
    Serial.print(pressure * 100);
    Serial.print("}");
    Serial.println("],\"source\":{\"label\":\"pesky.IMU\"}}]}");
    
      digitalWrite(myLed, !digitalRead(myLed)); 
      count = millis(); 
      sumCount = 0;
      sum = 0;    
    }
}

//===================================================================================================================
//====== Sentral parameter management functions
//===================================================================================================================

void EM7180_set_gyro_FS (uint16_t gyro_fs)
{
  uint8_t bytes[4], STAT;
  bytes[0] = gyro_fs & (0xFF);
  bytes[1] = (gyro_fs >> 8) & (0xFF);
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Gyro LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Gyro MSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Unused
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Unused

  // Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCB);

  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(STAT==0xCB)) {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_mag_acc_FS (uint16_t mag_fs, uint16_t acc_fs) {
  uint8_t bytes[4], STAT;
  bytes[0] = mag_fs & (0xFF);
  bytes[1] = (mag_fs >> 8) & (0xFF);
  bytes[2] = acc_fs & (0xFF);
  bytes[3] = (acc_fs >> 8) & (0xFF);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Mag LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Mag MSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Acc LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Acc MSB

  // Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCA);

  //Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(STAT==0xCA)) {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_integer_param (uint8_t param, uint32_t param_val)
{
  uint8_t bytes[4], STAT;
  bytes[0] = param_val & (0xFF);
  bytes[1] = (param_val >> 8) & (0xFF);
  bytes[2] = (param_val >> 16) & (0xFF);
  bytes[3] = (param_val >> 24) & (0xFF);
 
  // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  param = param | 0x80;
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);

  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(STAT==param)) {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_float_param (uint8_t param, float param_val) {
  uint8_t bytes[4], STAT;
  float_to_bytes (param_val, &bytes[0]);

  // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  param = param | 0x80;
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);

  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(STAT==param)) {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_WS_params()
{
  uint8_t param = 1;
  uint8_t STAT;
  // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  param = param | 0x80;
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, WS_params.Sen_param[0][0]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, WS_params.Sen_param[0][1]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, WS_params.Sen_param[0][2]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, WS_params.Sen_param[0][3]);
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);

  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(STAT==param))
  {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  for(uint8_t i=1; i<35; i++)
  {
    param = (i+1) | 0x80;
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, WS_params.Sen_param[i][0]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, WS_params.Sen_param[i][1]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, WS_params.Sen_param[i][2]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, WS_params.Sen_param[i][3]);
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
    
    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(STAT==param))
    {
      STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);
}

void EM7180_get_WS_params()
{
  uint8_t param = 1;
  uint8_t STAT;

  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
  delay(10);
  
  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);
  delay(10);

   // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(STAT==param))
  {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  
  // Parameter is the decimal value with the MSB set low (default) to indicate a paramter read processs
  WS_params.Sen_param[0][0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
  WS_params.Sen_param[0][1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
  WS_params.Sen_param[0][2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
  WS_params.Sen_param[0][3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);

  for(uint8_t i=1; i<35; i++)
  {
    param = (i+1);
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
    delay(10);
    
    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(STAT==param))
    {
      STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    WS_params.Sen_param[i][0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    WS_params.Sen_param[i][1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    WS_params.Sen_param[i][2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    WS_params.Sen_param[i][3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);

  // Re-start algorithm
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00);
}

void WS_PassThroughMode()
{
  uint8_t stat = 0;
  
  // First put SENtral in standby mode
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x01);
  delay(5);
  
  // Place SENtral in pass-through mode
  writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x01);
  delay(5);
  stat = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
  while(!(stat & 0x01))
  {
    stat = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
    delay(5);
  }
}

void WS_Resume()
{
  uint8_t stat = 0;
  
  // Cancel pass-through mode
  writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x00);
  delay(5);
  stat = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
  while((stat & 0x01))
  {
    stat = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
    delay(5);
  }

  // Re-start algorithm
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00);
  delay(5);
  stat = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
  while((stat & 0x01))
  {
    stat = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
    delay(5);
  }
}

bool checkSenParams()
{
  uint8_t data[142];
  uint8_t x;
  M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x80, 14, &data[128]); // Page 255
  delay(100);
  M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x00, 128, &data[0]); // Page 254
  delay(100);
  if(SerialDebug){
		for(x=0;x<142;x++){
			Serial.print(data[x], HEX);
			Serial.print(",");
		}
    Serial.println();
	}
  if(data[0] == 0x2A && data[1] == 0x65){
    //a valid warm start found
    if(SerialDebug)Serial.println("Warm Start found");
    return true;
  }else{
    if(SerialDebug)Serial.println("Warm Start not found");
    return false;
  }
  
}

void readSenParams()
{
  uint8_t data[142];
  uint8_t paramnum;
  
  M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x80, 14, &data[128]); // Page 255
  delay(100);
  M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x00, 128, &data[0]); // Page 254
  for (paramnum = 0; paramnum < 35; paramnum++) // 35 parameters
  {
    for (uint8_t i= 0; i < 4; i++)
    {
      WS_params.Sen_param[paramnum][i] = data[(paramnum*4 + i + 2)];
    }
  }
}

void writeSenParams()
{
  uint8_t data[142];
  data[0]=42;
  data[1]=101;
  uint8_t paramnum;
  for (paramnum = 0; paramnum < 35; paramnum++) // 35 parameters
  {
    for (uint8_t i= 0; i < 4; i++)
    {
      data[(paramnum*4 + i + 2)] = WS_params.Sen_param[paramnum][i];
    }
  }
  if(SerialDebug){
		uint8_t x;
		for(x=0;x<142;x++){
			Serial.print(data[x], HEX);
			Serial.print(",");
		}
    Serial.println();
	}
  M24512DFMwriteBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x80, 14, &data[128]); // Page 255
  delay(100);
  M24512DFMwriteBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x00, 128, &data[0]); // Page 254
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

float uint32_reg_to_float (uint8_t *buf)
{
  union {
    uint32_t ui32;
    float f;
  } u;

  u.ui32 =     (((uint32_t)buf[0]) +
               (((uint32_t)buf[1]) <<  8) +
               (((uint32_t)buf[2]) << 16) +
               (((uint32_t)buf[3]) << 24));
  return u.f;
}

void float_to_bytes (float param_val, uint8_t *buf) {
  union {
    float f;
    uint8_t comp[sizeof(float)];
  } u;
  u.f = param_val;
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = u.comp[i];
  }
  //Convert to LITTLE ENDIAN
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = buf[(sizeof(float)-1) - i];
  }
}

void readSENtralQuatData(float * destination)
{
  uint8_t rawData[16];  // x/y/z quaternion register data stored here
  readBytes(EM7180_ADDRESS, EM7180_QX, 16, &rawData[0]);       // Read the sixteen raw data registers into data array
  destination[0] = uint32_reg_to_float (&rawData[0]);
  destination[1] = uint32_reg_to_float (&rawData[4]);
  destination[2] = uint32_reg_to_float (&rawData[8]);
  destination[3] = uint32_reg_to_float (&rawData[12]);  // SENtral stores quats as qx, qy, qz, q0!

}

void readSENtralAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(EM7180_ADDRESS, EM7180_AX, 6, &rawData[0]);       // Read the six raw data registers into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void readSENtralGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(EM7180_ADDRESS, EM7180_GX, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void readSENtralMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(EM7180_ADDRESS, EM7180_MX, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

int16_t readSENtralBaroData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(EM7180_ADDRESS, EM7180_Baro, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
}

int16_t readSENtralTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(EM7180_ADDRESS, EM7180_Temp, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
}


//===================================================================================================================
//====== I2C Communication Support Functions
//===================================================================================================================

// I2C communication with the M24512DFM EEPROM is a little different from I2C communication with the usual motion sensor
// since the address is defined by two bytes

void M24512DFMwriteByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t  data)
{
	Wire.beginTransmission(device_address);   // Initialize the Tx buffer
	Wire.write(data_address1);                // Put slave register address in Tx buffer
	Wire.write(data_address2);                // Put slave register address in Tx buffer
	Wire.write(data);                         // Put data in Tx buffer
	Wire.endTransmission();                   // Send the Tx buffer
}

void M24512DFMwriteBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{
  if(count > 128)
  {
    count = 128;
    if(SerialDebug)Serial.print("Page count cannot be more than 128 bytes!");
  }
  Wire.beginTransmission(device_address);   // Initialize the Tx buffer
  Wire.write(data_address1);                // Put slave register address in Tx buffer
  Wire.write(data_address2);                // Put slave register address in Tx buffer
  for(uint8_t i=0; i < count; i++)
  {
    Wire.write(dest[i]);                    // Put data in Tx buffer
  }
  if(SerialDebug)Serial.print("I2C: ");
  if(SerialDebug)Serial.println(Wire.endTransmission());                   // Send the Tx buffer
}

uint8_t M24512DFMreadByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(device_address);         // Initialize the Tx buffer
	Wire.write(data_address1);                      // Put slave register address in Tx buffer
	Wire.write(data_address2);                      // Put slave register address in Tx buffer
	Wire.endTransmission(false);               // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(device_address, (size_t)1);   // Read one byte from slave register address 
	data = Wire.read();                             // Fill Rx buffer with result
	return data;                                    // Return data read from slave register
}

void M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(device_address);            // Initialize the Tx buffer
  Wire.write(data_address1);                         // Put slave register address in Tx buffer
  Wire.write(data_address2);                         // Put slave register address in Tx buffer
  Wire.endTransmission(false);                  // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(device_address, (size_t)count);  // Read bytes from slave register address 
  while (Wire.available())
  {
    dest[i++] = Wire.read();
  }                                                   // Put read results in the Rx buffer
}


// I2C read/write functions for the MPU9250 and AK8963 sensors

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);            // Initialize the Tx buffer
	Wire.write(subAddress);                     // Put slave register address in Tx buffer
	Wire.endTransmission(false);           // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
  Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
	while (Wire.available())
	{
    dest[i++] = Wire.read();
  }                                           // Put read results in the Rx buffer
}
