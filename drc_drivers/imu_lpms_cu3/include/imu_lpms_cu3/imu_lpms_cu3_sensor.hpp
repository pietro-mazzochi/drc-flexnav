/*
works with premium IMU (experimental) and lpresearch lpms-cu3
VERSION 0.240924
SENAI DTI
*/

#ifndef DRC_FLEXNAV_SENSORS__IMU_LPMS_CU3_SENSOR_HPP_
#define DRC_FLEXNAV_SENSORS__IMU_LPMS_CU3_SENSOR_HPP_

extern "C"
{
#include <signal.h>
#include <termios.h>
#include <fcntl.h> // Contains file controls
}

#include <cstdio>
#include <cstdlib>
#include <string>
#include <cstring>

/* to ACTIVATE debug messages output from IMU  */
// #define IMUDEBUG
/* ******************************************* */

#ifndef __DEBUG__
#define __DEBUG__

#ifdef IMUDEBUG
#define debug(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)
#else
#define debug(fmt, ...) ((void)0)
#endif

#endif

/* to use with json print 
const char *idACCRAW = "ACCRAW";
const char *idACCEL = "ACCEL";
const char *idgyro_raw = "gyro_raw";
const char *idgyro_bias = "gyro_bias";
const char *idGYRO = "GYRO";
const char *idmag_raw = "mag_raw";
const char *idMAG = "MAG";
const char *idANGVEL = "ANGVEL";
const char *idQUAT = "QUAT";
const char *idEULER = "EULER";
const char *idLINACC = "LINACC";
const char *idPRESS = "PRESS";
const char *idALT = "ALT";
const char *idTEMP = "TEMP";
const char *idBufferCount = "BuffCount";
const char *idErrorCount = "ErrorCount";
const char *idTransmitData = "TransmitData";
const char *idPrecision = "Precision";
*/

typedef enum
{
    FLEXNAV_IMU_OPERATING_MODE_COMMAND = 0,
    FLEXNAV_IMU_OPERATING_MODE_STREAM,
    FLEXNAV_IMU_OPERATING_MODE_SIZE
} IMU_operating_mode_t;

namespace drc_flexnav_sensors
{

    class Inertial_measurement_unit_and_compass
    {
    public:
        Inertial_measurement_unit_and_compass(void);
        bool init(const int ID = -1, const std::string comPort = "/dev/ttyUSB0", const uint32_t baudRate = B921600);
        IMU_operating_mode_t get_operating_mode(void);
        bool set_operating_mode(const IMU_operating_mode_t mode);
        bool set_stream_frequency(const uint32_t frequency);
        bool imu_ok(void); // return true if imu_serial_port_ is open
        bool stop(void);   // stop and close imu serial port
        bool flush(void);  // flush IMU buffer, erasing data IN and OUT
        bool set_id(const int idPar);
        int get_serial_port(void);  // get current serial port number
        // get a response from IMU, storing data in buffer. Parameter is timeout in x 10us
        int get_response(const uint16_t timeout = 20);
        // send a packet or command to IMU. When a packetData is used, the packetDataSize must be > 0
        bool send_packet(const uint16_t command, const uint16_t packetDataSize = 0, uint8_t *packetData = NULL, const int idToSend = -1);
        // send a 16 bits command to IMU.
        bool send_command(const uint16_t command);
        // send a command to imu, with a 32bits parameter, returning true if success
        bool send_command(const uint16_t command, const uint32_t value);
        // debug the buffer in HEX position
        void print_buffer(void);
        // return TRUE if detected IMU in command mode.
        bool in_command_mode(void);
        // set the transmit data flag, choosing the transmit IMU data.
        bool set_transmit_data(const uint32_t transmitDataFlag);
        // unset the transmit data flag, disabling the transmit IMU data.
        bool unset_transmit_data(const uint32_t transmitDataFlag);
        // print the transmit data configuration

        // void print_IMU_transmit_data(FILE *fOutput = stdout);
        // return all data received from IMU in JSON format.
        //        void toJson(char *str, const bool printStatus = false);
        //        // print all data received from IMU in JSON format.
        //        void printJsonIMUData(FILE *stdOut = stdout);

        bool set_32bits_precision(const bool precision = true);
        // get the current imu_transmit_data_ status
        uint32_t get_current_transmit_data(void);
        // method to request IMU data when in command mode.
        bool get_data(void);
        // method to run in main loop or in a task
        bool get_stream_loop(void);
        // method to detect ID (use if only 1 IMU is connected on tty)
        int detect_id(void);

        bool set_gyro_out_in_radians(uint32_t value);
        bool reset_alignment_offset(void);
        bool set_accelerometer_range(uint32_t value);
        bool set_filter_mode(uint32_t value);

        void set_accel_raw_x_factor(const float v);
        void set_accel_raw_y_factor(const float v);
        void set_accel_raw_z_factor(const float v);
        void set_accel_x_factor(const float v);
        void set_accel_y_factor(const float v);
        void set_accel_z_factor(const float v);
        void set_gyro_raw_x_factor(const float v);
        void set_gyro_raw_y_factor(const float v);
        void set_gyro_raw_z_factor(const float v);
        void set_gyro_x_factor(const float v);
        void set_gyro_y_factor(const float v);
        void set_gyro_z_factor(const float v);
        void set_gyro_bias_x_factor(const float v);
        void set_gyro_bias_y_factor(const float v);
        void set_gyro_bias_z_factor(const float v);
        void set_mag_raw_x_factor(const float v);
        void set_mag_raw_y_factor(const float v);
        void set_mag_raw_z_factor(const float v);
        void set_mag_x_factor(const float v);
        void set_mag_y_factor(const float v);
        void set_mag_z_factor(const float v);
        void set_angular_vel_x_factor(const float v);
        void set_angular_vel_y_factor(const float v);
        void set_angular_vel_z_factor(const float v);
        void set_quaternion_x_factor(const float v);
        void set_quaternion_y_factor(const float v);
        void set_quaternion_z_factor(const float v);
        void set_quaternion_w_factor(const float v);
        void set_euler_x_factor(const float v);
        void set_euler_y_factor(const float v);
        void set_euler_z_factor(const float v);
        void set_linear_accel_x_factor(const float v);
        void set_linear_accel_y_factor(const float v);
        void set_linear_accel_z_factor(const float v);
        void set_altitude_factor(const float v);
        void set_pressure_factor(const float v);
        void set_temperature_factor(const float v);

        uint32_t get_error_count(void); // get the buffer error count
        uint32_t get_timestamp(void);  // return last timestamp message received from IMU.
        float get_accel_raw_x(void);     // return x coordinate parameter from Accelerometer Raw data
        float get_accel_raw_y(void);     // return y coordinate parameter from Accelerometer Raw data
        float get_accel_raw_z(void);     // return z coordinate parameter from Accelerometer Raw data
        float get_accel_x(void);        // return x coordinate parameter from Accelerometer
        float get_accel_y(void);        // return y coordinate parameter from Accelerometer
        float get_accel_z(void);        // return z coordinate parameter from Accelerometer
        float get_gyro_raw_x(void);      // return x coordinate parameter from Gyroscope Raw data
        float get_gyro_raw_y(void);      // return y coordinate parameter from Gyroscope Raw data
        float get_gyro_raw_z(void);      // return z coordinate parameter from Gyroscope Raw data
        float get_gyro_x(void);         // return x coordinate parameter from Gyroscope
        float get_gyro_y(void);         // return y coordinate parameter from Gyroscope
        float get_gyro_z(void);         // return z coordinate parameter from Gyroscope
        float get_gyro_bias_x(void);     // return x coordinate parameter from Gyroscope Bias
        float get_gyro_bias_y(void);     // return y coordinate parameter from Gyroscope Bias
        float get_gyro_bias_z(void);     // return z coordinate parameter from Gyroscope Bias
        float get_mag_raw_x(void);       // return x coordinate parameter from Magnetometer Raw data
        float get_mag_raw_y(void);       // return y coordinate parameter from Magnetometer Raw data
        float get_mag_raw_z(void);       // return z coordinate parameter from Magnetometer Raw data
        float get_mag_x(void);          // return x coordinate parameter from Magnetometer
        float get_mag_y(void);          // return y coordinate parameter from Magnetometer
        float get_mag_z(void);          // return z coordinate parameter from Magnetometer
        float get_angular_vel_x(void);   // return x coordinate parameter from Angular Vel
        float get_angular_vel_y(void);   // return y coordinate parameter from Angular Vel
        float get_angular_vel_z(void);   // return z coordinate parameter from Angular Vel
        float get_quaternion_x(void);   // return x coordinate parameter from Quaternion
        float get_quaternion_y(void);   // return y coordinate parameter from Quaternion
        float get_quaternion_z(void);   // return z coordinate parameter from Quaternion
        float get_quaternion_w(void);   // return w coordinate parameter from Quaternion
        float get_euler_x(void);        // return x coordinate parameter from Euler
        float get_euler_y(void);        // return y coordinate parameter from Euler
        float get_euler_z(void);        // return z coordinate parameter from Euler
        float get_linear_accel_x(void);  // return x coordinate parameter from linear_accel
        float get_linear_accel_y(void);  // return y coordinate parameter from linear_accel
        float get_linear_accel_z(void);  // return z coordinate parameter from linear_accel
        float get_altitude(void);      // return Altitude
        float get_pressure(void);      // return Pressure
        float get_temperature(void);   // return Temperature

    private:
        bool imu_data_double_precision_mode_ = false;
        uint32_t imu_transmit_data_ = 0x00;
        uint32_t imu_config_ = 0xffff;
        int identifier_ = -1;
        uint32_t error_counter_ = 0;
        uint32_t imu_command_mode_ = 0;
        bool imu_ack_received_ = false;
        uint8_t imu_stream_frequency_ = 0;
        uint8_t imu_max_send_retries_ = 50;
        bool imu_flag_detect_id_ = false;

        struct packetStr
        {
            uint32_t timestamp;
            float accel_raw[3];
            float accel[3];
            float gyro_raw[3];
            float gyro[3];
            float gyro_bias[3];
            float mag_raw[3];
            float mag[3];
            float angular_vel[3];
            float quaternion[4];
            float euler[3];
            float linear_accel[3];
            float altitude;
            float pressure;
            float temperature;
        } imu_packet_data, imu_factor;

        int imu_serial_port_;

        // insert a byte into buffer FIFO
        bool buf_in(const uint8_t byte);
        // read (removing) a byte from buffer FIFO
        uint8_t buf_out(void);
        // read a byte from buffer pos, storing result on *data. Return false on error.
        bool buf_pos(uint8_t *data, const int pos);
        // get 32 bits from buffer pos
        int32_t get_32bits_integer_from(const int pos);

        // get 16 bits int from buffer pos
        int16_t get_16bits_integer_from(const int pos);

        // get 32 bits float from buffer pos
        float get_32bits_float_from(const int pos);

        // return bytes occupeds in buffer
        int get_buffer_size();

        // initializate the IMU serial port communication. Pass the port name (ex: "/dev/ttyUSB0") and baud rate (default 921600 bps). Return true if success.
        bool serial_init(const std::string port_name, const speed_t _baudrate = B921600);

        // write into serial port
        bool write_serial(uint8_t *buff, const int size = 1); // write buffer on serial port

        // get a command from first frame in buffer
        uint16_t get_frame_command(void); // extract a command from a received frame

        // get the size of data packet inside the first frame on buffer
        int get_frame_length(void); // return frame length

        // remove a frame from buffer
        int remove_frame_from_buffer(void);

        // get 32 bits uint packet content from first frame in buffer
        uint32_t get_frame_data_32bits(void); // get 32 bits packet content from a frame

        // get a float vector from buffer position pos, dimension times, storing in destVect vector.
        int get_float_vector(float destVect[], int pos, int const dimensions = 3);

        // get an int16 vector from buffer position pos, dimension times, storing in destVect vector.
        int get_int16_vector(float *destVect, int pos, const int dimensions = 3);

        /* internal use, for json convertion */
        bool parse_response(void); // read input buffer and parse response

        bool parse_get_transmit_data(void);
        bool parse_get_stream_freq(void);

        bool parse_get_id(void);

        bool parse_get_precision(void);

        bool parse_get_sensor_status(void);

        bool parse_imu_data(void);

        float factor_apply(const float v1, const float factor);
        bool read_ack(void);

        // Store the final position of 1st frame detected in *finalPos.
        // return true if imuBuffer contains a valid frame.
        bool valid_frame(int *finalPos = NULL);

        speed_t compute_serial_baudrate_config(const unsigned int baudrate);
    };

}

#endif // DRC_FLEXNAV_SENSORS__IMU_LPMS_CU3_SENSOR_HPP_
