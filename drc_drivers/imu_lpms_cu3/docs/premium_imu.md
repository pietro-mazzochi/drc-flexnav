# Premium IMU definitions

Describes some definitions for an experimental IMU sensor named "Premium IMU".
This current ROS2 package is supose to work for both IMU sensors, LPMS and Premium.

## Parameters

**imuComPort**

Port used to connect on IMU / MAGNETOMETER set
Default: "/dev/ttyUSB0"

**imuId**

Id # used by IMU. Use -1 to auto detect.
Default: 1

**imuPrecision**

Precision mode. Integer mode. Use 0 for 16-bit integer mode (IMU). The data obtained is multiplied by a factor, resulting in a floating number. Use 1 for 32 bits float single precision mode (this option causes more communication bus occupancy)
Default: 1

**imuSpeed**

Data rate in bytes per second. Valid values are 57600, 115200, 230400, 460800, 500000, 576000, 921600, 1000000, 1152000, 1500000, 2000000, 2500000, 3000000, 3500000, 4000000.
Default: 1000000 (1Mbps)

**imuFreq**

Frequency to publish data in Hz. Allowed 1 to 500. 
Default: 100

--------------------------------------------------

**premimumIMU.hpp**

use

...
#include <premiumIMU.h>
...

PremiumIMU IMU;

...


## Methods in PremiumIMU class:
TODO: the headers of the class should be enough documentation for this!
### Constructor:
```c        
        bool init(int ID = -1, const char * comPort = "/dev/ttyUSB0", u_int32_t baudRate = B1000000);
        // initialize com port, returning false if a error occurs.

        // return true if imuSerialPort is onen
        bool imuOk() 

        // stop and close IMU serial port
        bool stop()

        // clean and flush buffer
        bool flush()

        // set a new ID to an IMU
        bool setId(int idPar);

        // get the current serial port
        int getSerialPort() // get current serial port number

        // get a response from IMU, storing data in buffer. Parameter is timeout in x 10us
        int getResponse(u_int16_t timeout = 20) // timeout x 10 us

        // send a packet or command to IMU. When a packetData is used, the packetDataSize must be > 0
        bool sendPacket(u_int16_t command, u_int16_t packetDataSize = 0, u_int8_t *packetData = NULL) 
        
        // send a 2 bytes command.
        bool sendCommand(u_int16_t command, uint32_t parameter) // parameter is optional

        // printf the buffer in HEX position, for debug porpouses
        void printBuffer(FILE * fOutput = stdout)

        // return TRUE if detected IMU in command mode.
        bool inCommandMode()

        // set the transmit data flag, choosing the transmit IMU data.
        bool setTransmitData(u_int32_t transmitDataFlag)

        // unset the transmit data flag, disabling the transmit IMU data.
        bool unsetTransmitData(u_int32_t transmitDataFlag)

        // print the transmit data configuration, for debug or log porpouses
        void printIMUTransmitData(FILE * fOutput = stdout)

        // return all data received from IMU in JSON format. 
        void toJson(char *str)

        // print all data received from IMU in JSON format. 
        void printJsonIMUData(FILE * stdOut = stdout)

        // set or unset the float mode / 32 bits data precision.
        bool set32bitsPrecision(bool precision = true)

        // get the flags of IMU data transmited. 

        u_int32_t getCurrentTransmitData() 
    
    ```

        returns must be compared with this flags:

                IMU_TRANSMIT_ACCEL_RAW_OUTPUT    
                IMU_TRANSMIT_ACCEL_OUTPUT        
                IMU_TRANSMIT_GYRO_RAW_OUTPUT     
                IMU_TRANSMIT_GYRO_BIAS_OUTPUT    
                IMU_TRANSMIT_GYRO_OUTPUT         
                IMU_TRANSMIT_MAG_RAW_OUTPUT      
                IMU_TRANSMIT_MAG_OUTPUT          
                IMU_TRANSMIT_ANG_VEL_OUTPUT      
                IMU_TRANSMIT_QUATERNION_OUTPUT   
                IMU_TRANSMIT_EULER_OUTPUT        
                IMU_TRANSMIT_LINEAR_ACCELL_OUTPUT
                IMU_TRANSMIT_PRESSURE_OUTPUT     
                IMU_TRANSMIT_ALTITUDE_OUTPUT     
                IMU_TRANSMIT_TEMP_OUTPUT         

``` c
        // get data when in command mode. Return true if data received.
        bool getData()

        // method to run in main loop or in a task when in sctream mode.
        bool getStreamLoop() // return true if some data received and parsed correctly

        u_int32_t getErrorCount();      // get the buffer error count 
        u_int32_t getTimeStamp();       // return last timestamp message received from IMU.

        // return coordinates parameter from Accelerometer Raw data 
        float getAccelRawX();           
        float getAccelRawY();
        float getAccelRawZ()

        // return coordinate parameter from Accelerometer
        float getAccelX();
        float getAccelY();
        float getAccelZ();
        
        // return coordinate parameter from Gyroscope Raw data 
        float getGyroRawX(); 
        float getGyroRawY();
        float getGyroRawZ();

        // return x coordinate parameter from Gyroscope
        float getGyroX();
        float getGyroY();
        float getGyroZ();
  
        // return coordinate parameter from Gyroscope Bias
        float getGyroBiasX();
        float getGyroBiasY();
        float getGyroBiasZ();        

        // return paratemers from Magnetometer Raw Data
        float getMagRawX();
        float getMagRawY();
        float getMagRawZ();

        // return parameters from Magnetometer
        float getMagX();
        float getMagY();
        float getMagZ();

        // return coordinate parameter from Angular Vel      
        float getAngularVelX();
        float getAngularVelY();
        float getAngularVelZ();

        // return coordinate parameter from Quaternion      
        float getQuaternionX();
        float getQuaternionY();
        float getQuaternionZ();
        float getQuaternionW();

        // return x coordinate parameter from Euler
        float getEulerX();
        float getEulerY();
        float getEulerZ();
        
        // return Linear Accel data
        float getLinearAccelX();
        float getLinearAccelY();
        float getLinearAccelZ();

        // return Altitude
        float getAltitude();
        float getPressure();
        float getTemperature();

        void setAccelRawXFactor(float v)     // set adjust factor, used to divide raw data when use get___ method. Default value is 1000; 
        void setAccelRawYFactor(float v)     // set adjust factor, used to divide raw data when use get___ method. Default value is 1000; 
        void setAccelRawZFactor(float v)     // set adjust factor, used to divide raw data when use get___ method. Default value is 1000; 
        void setAccelXFactor(float v)        // set adjust factor, used to divide raw data when use get___ method. Default value is 1000; 
        void setAccelYFactor(float v)        // set adjust factor, used to divide raw data when use get___ method. Default value is 1000; 
        void setAccelZFactor(float v)        // set adjust factor, used to divide raw data when use get___ method. Default value is 1000; 
        void setGyroRawXFactor(float v)      // set adjust factor, used to divide raw data when use get___ method. Default value is 10; 
        void setGyroRawYFactor(float v)      // set adjust factor, used to divide raw data when use get___ method. Default value is 10; 
        void setGyroRawZFactor(float v)      // set adjust factor, used to divide raw data when use get___ method. Default value is 10; 
        void setGyroXFactor(float v)         // set adjust factor, used to divide raw data when use get___ method. Default value is 10; 
        void setGyroYFactor(float v)         // set adjust factor, used to divide raw data when use get___ method. Default value is 10; 
        void setGyroZFactor(float v)         // set adjust factor, used to divide raw data when use get___ method. Default value is 10; 
        void setGyroBiasXFactor(float v)     // set adjust factor, used to divide raw data when use get___ method. Default value is 10; 
        void setGyroBiasYFactor(float v)     // set adjust factor, used to divide raw data when use get___ method. Default value is 10; 
        void setGyroBiasZFactor(float v)     // set adjust factor, used to divide raw data when use get___ method. Default value is 10; 
        void setMagRawXFactor(float v)       // set adjust factor, used to divide raw data when use get___ method. Default value is 100; 
        void setMagRawYFactor(float v)       // set adjust factor, used to divide raw data when use get___ method. Default value is 100; 
        void setMagRawZFactor(float v)       // set adjust factor, used to divide raw data when use get___ method. Default value is 100; 
        void setMagXFactor(float v)          // set adjust factor, used to divide raw data when use get___ method. Default value is 100; 
        void setMagYFactor(float v)          // set adjust factor, used to divide raw data when use get___ method. Default value is 100; 
        void setMagZFactor(float v)          // set adjust factor, used to divide raw data when use get___ method. Default value is 100; 
        void setAngularVelXFactor(float v)   // set adjust factor, used to divide raw data when use get___ method. Default value is 10; 
        void setAngularVelYFactor(float v)   // set adjust factor, used to divide raw data when use get___ method. Default value is 10; 
        void setAngularVelZFactor(float v)   // set adjust factor, used to divide raw data when use get___ method. Default value is 10; 
        void setQuaternionXFactor(float v)   // set adjust factor, used to divide raw data when use get___ method. Default value is 10000;
        void setQuaternionYFactor(float v)   // set adjust factor, used to divide raw data when use get___ method. Default value is 10000;
        void setQuaternionZFactor(float v)   // set adjust factor, used to divide raw data when use get___ method. Default value is 10000;
        void setQuaternionWFactor(float v)   // set adjust factor, used to divide raw data when use get___ method. Default value is 10000;
        void setEulerXFactor(float v)        // set adjust factor, used to divide raw data when use get___ method. Default value is 100; 
        void setEulerYFactor(float v)        // set adjust factor, used to divide raw data when use get___ method. Default value is 100; 
        void setEulerZFactor(float v)        // set adjust factor, used to divide raw data when use get___ method. Default value is 100; 
        void setLinearAccelXFactor(float v   // set adjust factor, used to divide raw data when use get___ method. Default value is 1000; 
        void setLinearAccelYFactor(float v   // set adjust factor, used to divide raw data when use get___ method. Default value is 1000; 
        void setLinearAccelZFactor(float v   // set adjust factor, used to divide raw data when use get___ method. Default value is 1000; 
        void setAltitudeFactor(float v)      // set adjust factor, used to divide raw data when use get___ method. Default value is 10; 
        void setPressureFactor(float v)      // set adjust factor, used to divide raw data when use get___ method. Default value is 100; 
        void setTemperatureFactor(float v)   // set adjust factor, used to divide raw data when use get___ method. Default value is 100; 

        int detectId(); // it may take a few seconds

```
### CONSTANTS

  - IMU_TRANSMIT_ACCEL_RAW_OUTPUT      : 0b00000000000000001
  - IMU_TRANSMIT_ACCEL_OUTPUT          : 0b00000000000000010
  - IMU_TRANSMIT_GYRO_RAW_OUTPUT       : 0b00000000000001000
  - IMU_TRANSMIT_GYRO_BIAS_OUTPUT      : 0b00000000000100000
  - IMU_TRANSMIT_GYRO_OUTPUT           : 0b00000000010000000
  - IMU_TRANSMIT_MAG_RAW_OUTPUT        : 0b00000000100000000
  - IMU_TRANSMIT_MAG_OUTPUT            : 0b00000001000000000
  - IMU_TRANSMIT_ANG_VEL_OUTPUT        : 0b00000010000000000
  - IMU_TRANSMIT_QUATERNION_OUTPUT     : 0b00000100000000000
  - IMU_TRANSMIT_EULER_OUTPUT          : 0b00001000000000000
  - IMU_TRANSMIT_LINEAR_ACCELL_OUTPUT  : 0b00010000000000000
  - IMU_TRANSMIT_PRESSURE_OUTPUT       : 0b00100000000000000
  - IMU_TRANSMIT_ALTITUDE_OUTPUT       : 0b01000000000000000
  - IMU_TRANSMIT_TEMP_OUTPUT           : 0b10000000000000000
  - IMU_PACKET_START_BYTE              : 0x3A
  - IMU_PACKET_END1_BYTE               : 0x0D
  - IMU_PACKET_END2_BYTE               : 0x0A
  - IMU_GET_CONFIG                     : 0x1F
  - IMU_RESTORE_DEFAULT                : 0x05
  - IMU_REPLY_ACK                      : 0x00
  - IMU_REPLY_NACK                     : 0x01
  - IMU_WRITE_REGS                     : 0x04
  - IMU_RESTORE_DEFAULTS               : 0x05
  - IMU_GOTO_COMMAND_MODE              : 0x06
  - IMU_GOTO_STREAM_MODE               : 0x07
  - IMU_GET_SENSOR_STATUS              : 0x08
  - IMU_GET_DATA                       : 0x09
  - IMU_GET_SENSOR_MODEL               : 0x14
  - IMU_FIRM_INFO                      : 0x15
  - IMU_SERIAL_NO                      : 0x16
  - IMU_FILTER_VER                     : 0x17
  - IMU_SET_TRANSMIT_DATA              : 0x1E
  - IMU_GET_TRANSMIT_DATA              : 0x1F
  - IMU_SET_ID                         : 0x20
  - IMU_GET_ID                         : 0x21
  - IMU_SET_STREAM_FREQ                : 0x22
  - IMU_GET_STREAM_FREQ                : 0x23
  - IMU_SET_DEGRAD_OUT                 : 0x24
  - IMU_GET_DEGRAD_OUT                 : 0x25
  - IMU_SET_ORIENT_OFFSET              : 0x26
  - IMU_GET_ORIENT_OFFSET              : 0x27
  - IMU_SET_ACC_RANGE                  : 0x32
  - IMU_GET_ACC_RANGE                  : 0x33
  - IMU_SET_GYRO_RANGE                 : 0x3C
  - IMU_GET_GYRO_RANGE                 : 0x3D
  - IMU_START_GYRO_CALIB               : 0x3E
  - IMU_SET_GYRO_AUTOCALIB             : 0x40
  - IMU_GET_GYRO_AUTOCALIB             : 0x41
  - IMU_SET_MAG_RANGE                  : 0x46
  - IMU_GET_MAG_RANGE                  : 0x47
  - IMU_START_MAG_CALIB                : 0x54
  - IMU_STOP_MAG_CALIB                 : 0x55
  - IMU_SET_MAG_CAL_TIMEOUT            : 0x56
  - IMU_GET_MAG_CAL_TIMEOUT            : 0x57
  - IMU_SET_FILTER_MODE                : 0x5A
  - IMU_GET_FILTER_MODE                : 0x5B
  - IMU_SET_UART_BAUDRATE              : 0x82
  - IMU_GET_UART_BAUDRATE              : 0x83
  - IMU_SET_UART_FORMAT                : 0x84
  - IMU_GET_UART_FORMAT                : 0x85
  - IMU_SET_LPBUS_PRECISION            : 0x88
  - IMU_GET_LPBUS_PRECISION            : 0x89
  - IMU_SET_TIMESTAMP                  : 0x88
  - IMU_STREAM_FREQ_5HZ                : 5
  - IMU_STREAM_FREQ_10HZ               : 10
  - IMU_STREAM_FREQ_50HZ               : 50
  - IMU_STREAM_FREQ_100HZ              : 100
  - IMU_STREAM_FREQ_250HZ              : 250
  - IMU_STREAM_FREQ_500HZ              : 500

  json constants ids

  - "ACCRAW"          Accell Raw Data
  - "ACCEL"           Accell calibrated data
  - "GYRORAW"         Gyroscope Raw Data
  - "GYROBIAS"        Gyroscope Bias
  - "GYRO"            Gyroscope caibrated data
  - "MAGRAW"          Magnetometer Raw Data
  - "MAG"             Magnetometer Data
  - "ANGVEL"          Angular Velocity
  - "QUAT"            Quaternion
  - "EULER"           Euler Angles
  - "LINACC"          Linear Acceleration
  - "PRESS"           Pressure
  - "ALT"             Altimeter
  - "TEMP"            Temperature
  - "BuffCount"       Bytes in buffer
  - "ErrorCount"      Count of errors
  - "TransmitData"    bitwise or over IMU_TRANSMIT_... values. 
  - "Precision"       false (16 bits integer IMU->LIB) or true (32 bits float IMU->LIB) 
