#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"
#include "wdt.h"
#include "Wire.h"

#define ARR_SIZE 64

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // 
VectorFloat gravity;    // Gravity vector
float ypr[3];           // Array to store yaw pitch and roll values [0]- yaw , [1]- pitch , [2]- roll 

MPU6050 mpu ;           // Accelorometer Object
HMC5883L mag ;          // Magnetometer Object

int16_t mag_x[ARR_SIZE] , mag_y[ARR_SIZE] , mx , my , mz ; // Arrays and variables for 

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() {
  Wire.begin() ;                     //Beginning communications
  Serial.begin(9600) ;               //Opening Serial.port
  while (!Serial) ;                  //Only needed for native USB, comment after testing
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
  for (uint8_t i = 0 ; i<ARR_SIZE ; i++){        //
    mag_x[i]= NULL;                              //Initialises array values to null
    mag_y[i]= NULL;                              //
  }
  
}

void loop() {                       
  uint8_t tr1 = 0, tr2 = 0;              //Variables to access indices of the arrays
  const uint8_t tr3 = 0 ;                //Constant variable pointing at top of the array    
  while(1){
     // reset interrupt flag and get INT_STATUS byte             
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
    }
    delay(100) ;                //Waits for the specific amount of milliseconds specified in the argument
    wdt_reset() ;               
    mpu.dmpGetQuaternion(&q, fifoBuffer);          //Gets the quaternion values from the accelerometer 
    mpu.dmpGetGravity(&gravity, &q);               //Gets gravity value
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);     //Gets Yaw pitch and roll values in radians
    mag.getHeading(&mx,&my ,&mz) ;                 //Gets raw value from magnetometer
    mag_x[tr1]= ((mx*cos(ypr[1])) + (my*sin(ypr[2])*cos(ypr[1])) + (mz*cos(ypr[2])*sin(ypr[1]))) ;       //Calibrates raw value from angles given by accelerometer
    mag_y[tr1]= ( (my * cos(ypr[2]))- (mz * sin(ypr[2])) ) ;
    if (tr1<ARR_SIZE && tr2<ARR_SIZE) {tr1 ++ ;tr2++ ;}                 //goes to next index 
    else {tr1 = 0;}
    uint8_t temp = Serial.read() ;              //reads temp for printing 
    String bufferM = "" ;                       //string variable for printing the output
    if (temp == 63){                            //ASCII value for '?' is 063
        for(uint8_t i= tr1+1 ; i<tr2 ; i++){       //Storing values in buffer  
          bufferM=bufferM+String(mag_x[i]) +" "+ String(mag_y[i]) + "\n";             
        }
        for (uint8_t i = tr3 ; i<=tr2 ; i++ ){             
          bufferM=bufferM+String(mag_x[i]) +" "+ String(mag_y[i]) + "\n";
        }
        
        if (bufferM.length()>255) {                            //printing '!' size of buffer and printing buffer  
            bufferM = bufferM.substring(0,255) ;
            Serial.print('!'); Serial.print(String(255)) ;
            Serial.print(bufferM) ;
            bufferM = "" ;
        }else{
          Serial.print('!') ;Serial.print(" "); Serial.print(String(bufferM.length())); Serial.print(bufferM); 
          bufferM = "" ;
        }
        tr2 = 0 , tr1=0 ;              //resetting the indices 
        temp = 0 ;
    }
    
    
  }   
}
