#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"
#include "wdt.h"
#include "Wire.h"
#include <SoftwareSerial.h>
SoftwareSerial com( 3 , 4 );

#define ARR_SIZE 16
#define HR 3600000  
#define LIMIT 43200000

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
unsigned int del = 0 , diff;
char temp , x;
String bufferM ;
unsigned long Read_tme , Send_tme ;

Quaternion q;           //
VectorFloat gravity;    // Gravity vector
float ypr[3];           // Array to store yaw pitch and roll values [0]- yaw , [1]- pitch , [2]- roll

MPU6050 mpu ;           // Accelorometer Object
HMC5883L mag ;          // Magnetometer Object

int16_t mag_x[ARR_SIZE] , mag_y[ARR_SIZE] , mx , my , mz ; // Arrays and variables for

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high




void setup() {
  Wire.begin() ;                     //Beginning communications
  Serial.begin(9600) ;               //Opening Serial.port
  com.begin(9600);
  //while (!Serial) ;                  //Only needed for native USB, comment after testing
  mpu.initialize();                  //Initialising devices
  mag.initialize();                  //*

  wdt_enable(WDTO_8S) ;              //Enabling WDT(Watch Dog Timer)- Timeout = 8 seconds

  devStatus = mpu.dmpInitialize();   //Initialising Digital Motion Processing for 3D values. Returns 0 if connections are successful, else ! 0.

  if  (devStatus == 0) {
    mpu.setDMPEnabled(true) ;
    mpuIntStatus = mpu.getIntStatus() ;                     //After checking whether connections are correct, initialise FIFO and interrupt
    dmpReady = true ;
    packetSize = mpu.dmpGetFIFOPacketSize() ;
  }
  wdt_reset() ;                                    //Resets WDT to 0

  for (uint8_t i = 0 ; i < ARR_SIZE ; i++) {     //
    mag_x[i] = NULL;                             //Initialises array values to null
    mag_y[i] = NULL;                             //
  }
}


void loop() {

  uint8_t tr1 = 0, tr2 = 0;                                                                     //Variables to access indices of the arrays
  const uint8_t tr3 = 0 ;                                                                       //Constant variable pointing at top of the array

  while (1) {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;

    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02) {                                                             // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();                            // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    }
    wdt_disable() ;
    delay(45000) ;                                                                                 //Waits for the specific amount of milliseconds specified in the argument
    delay(45000) ;
    delay(45000) ;
    delay(45000) ;
    delay(45000) ;
    wdt_enable(WDTO_8S) ;
    wdt_reset() ;

    mpu.dmpGetQuaternion(&q, fifoBuffer);                                                       //Gets the quaternion values from the accelerometer
    mpu.dmpGetGravity(&gravity, &q);                                                            //Gets gravity value
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);                                                  //Gets Yaw pitch and roll values in radians
    mag.getHeading(&mx, &my , &mz) ;                                                            //Gets raw value from magnetometer
    mag_x[tr1] = ((mx * cos(ypr[1])) + (my * sin(ypr[2]) * cos(ypr[1])) + (mz * cos(ypr[2]) * sin(ypr[1]))) ; //Calibrates raw value from angles given by accelerometer
    mag_y[tr1] = ( (my * cos(ypr[2])) - (mz * sin(ypr[2])) ) ;
    temp = "";
    if (tr1 < ARR_SIZE && tr2 < ARR_SIZE) {
      tr1 ++ ;                                                                                 //goes to next index
      tr2++ ;                                                                                  //to assign pointer to the latest index for indexing data from new to old
    }

    else {
      Read_tme = millis() ;
      tr1 = 0;
      while (com.available() == 0) {};

      //String temp = com.readStringUntil(' ') ;                                                  //reads temp for printing
      temp = com.read();
      //char temp = Serial.read();
      bufferM = " " ;                                                                    //string variable for printing the output
      x = '?';                                                                              //hardcoring the value of question into a variable

      if ( temp == x  ) {                                                                        //to check if the read is a quetion mark
        Send_tme = millis() ;
        for (uint8_t i = tr1 + 1 ; i < tr2 ; i++) {
          bufferM = bufferM + String(mag_x[i]) + " " + String(mag_y[i]) + ":";                   //indexing both the arrays from the pointer to the end of the array and sending them into one array named buffer
        }

        for (uint8_t i = tr3 ; i <= tr2 ; i++ ) {
          bufferM = bufferM + String(mag_x[i]) + " " + String(mag_y[i]) + ',';                   //indexing both the arrays from beginig to the pointer into one array named buffer
        }
        diff = (unsigned int)( (Send_tme - Read_tme)/HR ) ;
        bufferM = String(diff)+bufferM ;
        if (bufferM.length() > 250) {                                                            //printing '!' size of buffer and printing buffer
          bufferM = bufferM.substring(0, 250) ;

          Serial.print('0x21'); Serial.print((unsigned char)0xFA) ;                               //serial print the data
          Serial.print(bufferM) ;
          com.print('!'); com.write((unsigned char)0xFA) ;                                        //com print the ! and size
          while (bufferM.length() < 250) {
            bufferM = bufferM + " " ;
          }
          com.print(bufferM) ;
          Serial.println() ;
          bufferM = "" ;                                                                         //clear the array
        }
        else {                                                                                   //if the data is less than 250 bytes
          int len = bufferM.length() ;                                                           //calculate the size of the array
          com.print('!') ; com.write( (unsigned char)len); com.print(bufferM);                   //com print the data
          Serial.print('!') ; Serial.print((unsigned char)len); Serial.print('\n'); Serial.print(bufferM); //serial print the data
          Serial.println() ;
          bufferM = "" ;                                                                         //clear the array
        }
        tr2 = 0 , tr1 = 0 ;                                                                      //resetting the indices
        temp = "" ;                                                                              //clear the temp value
      }
      wdt_disable();
      delay(del);
      wdt_enable(WDTO_8S) ;
      del = del + HR;
      if (del >= LIMIT) {
        del = 0;
      }
    }

  }
}
