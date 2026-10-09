#ifndef PREMIUM_IMU_CPP
#define PREMIUM_IMU_CPP

#include <iostream>

#include <imu_lpms_cu3/imu_lpms_cu3_sensor.hpp>
#include <imu_lpms_cu3/imu_lpms_cu3_definitions.hpp>

constexpr float GRAVITY = 9.80665f;

namespace drc_flexnav_sensors
{

    uint8_t imu_circular_buffer_[IMU_INPUT_BUFFER_SIZE];
    int imu_index_buffer_in_ = 0;
    int imu_index_buffer_out_ = 0;
    int imu_buffer_count_ = 0;

    Inertial_measurement_unit_and_compass::Inertial_measurement_unit_and_compass(void)
    {
    }

    // insert a byte into buffer FIFO
    bool Inertial_measurement_unit_and_compass::buf_in(const uint8_t byte)
    {
        if (imu_buffer_count_ >= IMU_INPUT_BUFFER_SIZE)
        {
            debug("\nBuffer overflow...");
            flush();
            return (false);
        }
        imu_circular_buffer_[imu_index_buffer_in_++] = byte;
        imu_index_buffer_in_ %= IMU_INPUT_BUFFER_SIZE;
        imu_buffer_count_++;
        return (true);
    }
    // read (removing) a byte from buffer FIFO
    uint8_t Inertial_measurement_unit_and_compass::buf_out(void)
    {
        uint8_t byte;
        if (imu_buffer_count_ <= 0)
            return (0);
        imu_buffer_count_--;
        byte = imu_circular_buffer_[imu_index_buffer_out_++];
        imu_index_buffer_out_ %= IMU_INPUT_BUFFER_SIZE;
        return (byte);
    }
    // read a byte from buffer pos, storing result on *data. Return false on error.
    bool Inertial_measurement_unit_and_compass::buf_pos(uint8_t *data, const int pos)
    {
        if (pos > imu_buffer_count_)
            return (false); // dont't have this position on circular buffer
        int index;
        index = imu_index_buffer_out_ + pos;
        index %= IMU_INPUT_BUFFER_SIZE;
        *data = imu_circular_buffer_[index];
        return (true);
    }

    // get 32 bits from buffer pos
    int32_t Inertial_measurement_unit_and_compass::get_32bits_integer_from(const int pos)
    {
        int32_t value = 0;
        uint8_t data[4];
        if (!buf_pos(&data[0], pos))
            return (false);
        if (!buf_pos(&data[1], pos + 1))
            return (false);
        if (!buf_pos(&data[2], pos + 2))
            return (false);
        if (!buf_pos(&data[3], pos + 3))
            return (false);
        memcpy(&value, data, 4);
        return (value);
    }

    // get 16 bits int from buffer pos
    int16_t Inertial_measurement_unit_and_compass::get_16bits_integer_from(const int pos)
    {
        int16_t value = 0;
        uint8_t data[2];
        if (!buf_pos(&data[0], pos))
            return (false);
        if (!buf_pos(&data[1], pos + 1))
            return (false);
        memcpy(&value, data, 2);
        return (value);
    }

    // get 32 bits float from buffer pos
    float Inertial_measurement_unit_and_compass::get_32bits_float_from(const int pos)
    {
        float value;
        int32_t aux;
        //            uint8_t data[4];
        aux = get_32bits_integer_from(pos);
        memcpy(&value, &aux, 4);
        return (value);
    }

    // return bytes occupeds in buffer
    int Inertial_measurement_unit_and_compass::get_buffer_size()
    {
        return (imu_buffer_count_);
    }

    // initializate the IMU serial port communication. Pass the port name (ex: "/dev/ttyUSB0") and baudrate_ rate (default 921600 bps). Return true if success.
    bool Inertial_measurement_unit_and_compass::serial_init(const std::string port_name, const speed_t _baudrate)
    {
        imu_serial_port_ = open(port_name.c_str(), O_RDWR | O_NONBLOCK);
        struct termios tty;
        if (tcgetattr(imu_serial_port_, &tty) != 0)
        {
            debug("Error %i from tcgetattr: %s\n", errno, strerror(errno));
            return false;
        }

        cfsetispeed(&tty, _baudrate);
        cfsetospeed(&tty, _baudrate);

        tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity
        tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication
        tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
        tty.c_cflag |= CS8;            // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control
                                       //            tty.c_cflag |= CRTSCTS; // Enable RTS/CTS hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;                                                        // Disable echo
        tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
        tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
        tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

        tty.c_cc[VTIME] = 0; // Wait for up to 0ms (0 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        // Save tty settings and check for error
        if (tcsetattr(imu_serial_port_, TCSANOW, &tty) != 0)
        {
            debug("Error %i from tcsetattr: %s\n", errno, strerror(errno));
            return false;
        }

        //          printf("c_cc:%s, c_flag:%x, c_iflag:%x, c_ispeed:%x, c_lflag:%x, c_line:%x, c_oflag:%x, c_ospeed:%x",
        //                 tty.c_cc, tty.c_cflag, tty.c_iflag, tty.c_ispeed, tty.c_lflag, tty.c_line, tty.c_oflag , tty.c_ospeed);
        //          getchar();
        tcflush(imu_serial_port_, TCIOFLUSH); 
        return true;
    }

    // write into serial port
    bool Inertial_measurement_unit_and_compass::write_serial(uint8_t *buff, const int size) // write buffer on serial port
    {
        if (imu_serial_port_ < 0 || write(imu_serial_port_, buff, size) <= 0)
        {
            debug("Fail to write on serial port -> IMU");
            return (false);
        }
        //            debug("\nSENDING: ");
        //            for (int i = 0; i < size; i ++) debug("[%02X]", buff[i]);
        return (true);
    }

    // get a command from first frame in buffer
    uint16_t Inertial_measurement_unit_and_compass::get_frame_command(void) // extract a command from a received frame
    {
        uint8_t datal, datah;
        if (!buf_pos(&datal, 3) || !buf_pos(&datah, 4))
            return (false);
        return ((int)datal + (int)datah * 256);
    }

    // get the size of data packet inside the first frame on buffer
    int Inertial_measurement_unit_and_compass::get_frame_length(void) // return frame length
    {
        uint8_t datal, datah;
        if (!buf_pos(&datal, 5) || !buf_pos(&datah, 6))
            return (false);
        return ((int)datal + (int)datah * 256);
    }

    // remove a frame from buffer
    int Inertial_measurement_unit_and_compass::remove_frame_from_buffer(void) // remove frame from buffer
    {
        int final_pos;
        if (!valid_frame(&final_pos)) // don't recognize data
        {
            error_counter_++;
            debug("\nFail to recognize data on serial buffer... flushing buffer.");
            flush();
            return (0);
        }
        if (final_pos <= imu_buffer_count_)
        {
            imu_buffer_count_ -= final_pos + 1;
            imu_index_buffer_out_ += final_pos + 1;
            imu_index_buffer_out_ %= IMU_INPUT_BUFFER_SIZE;
        }
        return (final_pos);
    }

    bool Inertial_measurement_unit_and_compass::set_gyro_out_in_radians(uint32_t value = 1) {
        //uint32_t value = 1; // https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1941864458/LPMS3+Command+List#SET_DEGRAD_OUTPUT
        return send_command(IMU_SET_DEGRAD_OUT, value);
    }

    bool Inertial_measurement_unit_and_compass::reset_alignment_offset(void) {
        uint32_t value = 0; // https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1941864458/LPMS3+Command+List#SET_ORIENTATION_OFFSET
        return send_command(IMU_SET_ORIENT_OFFSET, value);
    }

    bool Inertial_measurement_unit_and_compass::set_accelerometer_range(uint32_t value = 2) {
        // uint32_t value = 2; // https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1941864458/LPMS3+Command+List#SET_ACC_RANGE
        return send_command(IMU_SET_ACC_RANGE, value);
    }

    bool Inertial_measurement_unit_and_compass::set_filter_mode(uint32_t value = 1)
        {
        //https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1941864458/LPMS3+Command+List#SET_FILTER_MODE
        return send_command(IMU_SET_FILTER_MODE, value);
        }



    // get 32 bits uint packet content from first frame in buffer
    uint32_t Inertial_measurement_unit_and_compass::get_frame_data_32bits(void) // get 32 bits packet content from a frame
    {
        uint32_t value = 0;
        uint8_t data;
        int index = 10;
        while (index > 6)
        {
            if (!buf_pos(&data, index--))
                return (false);
            value *= 256;
            value += data;
        }
        return (value);
    }

    // get a float vector from buffer position pos, dimension times, storing in destVect vector.
    int Inertial_measurement_unit_and_compass::get_float_vector(float destVect[], int pos, int const dimensions)
    {
        int offset = 0;
        for (int i = 0; i < dimensions; i++)
        {
            destVect[i] = get_32bits_float_from(pos);
            pos += 4;
            offset += 4;
        }
        return (offset);
    }

    // get an int16 vector from buffer position pos, dimension times, storing in destVect vector.
    int Inertial_measurement_unit_and_compass::get_int16_vector(float *destVect, int pos, const int dimensions)
    {
        int offset = 0;
        for (int i = 0; i < dimensions; i++)
        {
            destVect[i] = get_16bits_integer_from(pos);
            pos += 2;
            offset += 2;
        }
        return (offset);
    }

    bool Inertial_measurement_unit_and_compass::parse_response(void) // read input buffer and parse response
    {
        uint8_t data;
        bool success = false;
        if (imu_buffer_count_ < IMU_MIN_FRAME_SIZE)
        {
            return (false);
        }
        while (imu_buffer_count_ >= IMU_MIN_FRAME_SIZE)
        {
            if (!buf_pos(&data, 0))
            {
                error_counter_++;
                debug("\nUnknow error...");
                flush();
                return (false);
            }
            if (data == IMU_PACKET_START_BYTE)
            {
                // have a packet in buffer begin. Validating...
                int idReceived = -1;
                if (valid_frame())
                {
                    uint32_t command = get_frame_command();
                    switch (command)
                    {
                    case IMU_REPLY_ACK:
                        imu_ack_received_ = true;
                        success = true;
                        break;
                    case IMU_REPLY_NACK:
                        imu_ack_received_ = false;
                        success = true;
                        break;
                    case IMU_GET_SENSOR_STATUS:
                        success = parse_get_sensor_status();
                        break;
                    case IMU_GET_DATA:
                        success = parse_imu_data();
                        break;
                    case IMU_GET_SENSOR_MODEL:
                    case IMU_FIRM_INFO:
                    case IMU_SERIAL_NO:
                    case IMU_FILTER_VER:
                        break;
                    case IMU_GET_LPBUS_PRECISION:
                        success = parse_get_precision();
                        break;
                    case IMU_GET_TRANSMIT_DATA:
                        success = parse_get_transmit_data();
                        break;
                    case IMU_GET_ID:
                        success = parse_get_id();
                        break;
                    case IMU_GET_STREAM_FREQ:
                        success = parse_get_stream_freq();
                        break;
                    case IMU_GET_DEGRAD_OUT:
                    case IMU_RESET_ORIENT_OFFSET:
                    case IMU_GET_ACC_RANGE:
                    case IMU_GET_GYRO_RANGE:
                    case IMU_GET_GYRO_AUTOCALIB:
                    case IMU_GET_MAG_RANGE:
                    case IMU_GET_MAG_CAL_TIMEOUT:
                    case IMU_GET_FILTER_MODE:
                    case IMU_GET_UART_BAUDRATE:
                    case IMU_GET_UART_FORMAT:
                        break;
                    default:
                        debug("\nUNKNOW PACK: ");
                    }
                    if (!success)
                    {
                        debug(" FAIL!!!!");
                        error_counter_++;
                    }
                    remove_frame_from_buffer();
                    return (true);
                }
                else
                {
                    return (false);
                }
            }
            else
            {
                debug("\nERROR, first byte is a %02X", data);
                error_counter_++;
#ifdef IMUDEBUG
                print_buffer();
#endif
                flush();
            }
        }
        return (false);
    }

    bool Inertial_measurement_unit_and_compass::parse_get_transmit_data(void)
    {
        debug("\nParsing list of transmited data ...");
        int size = get_frame_length();
        if (size != 4)
            return (false); // invalid frame size
        uint32_t value = get_frame_data_32bits();
        imu_transmit_data_ = value;
#ifdef IMUDEBUG
        print_IMU_transmit_data();
#endif
        return (true);
    }

    bool Inertial_measurement_unit_and_compass::parse_get_stream_freq(void)
    {
        debug("\nParsing Stream Frequency ...");
        int size = get_frame_length();
        if (size != 4)
            return (false); // invalid frame size
        uint32_t value = get_frame_data_32bits();
        imu_stream_frequency_ = value;
        return (true);
    }

    bool Inertial_measurement_unit_and_compass::parse_get_id(void)
    {
        debug("\nParsing ID detection: ");
        int size = get_frame_length();
        if (size != 4)
            return (false); // invalid frame size
        uint32_t value = get_frame_data_32bits();
        if (identifier_ > 0)
            identifier_ = value;
        debug("ID detected %d. ", identifier_);
        return (true);
    }

    bool Inertial_measurement_unit_and_compass::parse_get_precision(void)
    {
        debug("\nParsing data precision: ");
        int size = get_frame_length();
        if (size != 4)
            return (false); // invalid frame size
        uint32_t value = get_frame_data_32bits();
        imu_data_double_precision_mode_ = (value > 0);
        if (imu_data_double_precision_mode_)
            debug("float 32 bits.");
        else
            debug("integer 16 bits.");
        return (true);
    }

    bool Inertial_measurement_unit_and_compass::parse_get_sensor_status(void)
    {
        debug("\nParsing sensor status ...");
        int size = get_frame_length();
        if (size != 4)
            return (false); // invalid frame size
        uint32_t value = get_frame_data_32bits();
        if (value == 0)
            imu_command_mode_ = true;
        else
            imu_command_mode_ = false;
        return (true);
    }

    bool Inertial_measurement_unit_and_compass::parse_imu_data(void)
    {
        int size = get_frame_length();
        int sizeExpected = 0;
        if (IMU_TRANSMIT_ACCEL_RAW_OUTPUT & imu_transmit_data_)
            sizeExpected += 2 * 3;
        if (IMU_TRANSMIT_ACCEL_OUTPUT & imu_transmit_data_)
            sizeExpected += 2 * 3;
        if (IMU_TRANSMIT_GYRO_RAW_OUTPUT & imu_transmit_data_)
            sizeExpected += 2 * 3;
        if (IMU_TRANSMIT_GYRO_BIAS_OUTPUT & imu_transmit_data_)
            sizeExpected += 2 * 3;
        if (IMU_TRANSMIT_GYRO_OUTPUT & imu_transmit_data_)
            sizeExpected += 2 * 3;
        if (IMU_TRANSMIT_MAG_RAW_OUTPUT & imu_transmit_data_)
            sizeExpected += 2 * 3;
        if (IMU_TRANSMIT_MAG_OUTPUT & imu_transmit_data_)
            sizeExpected += 2 * 3;
        if (IMU_TRANSMIT_ANG_VEL_OUTPUT & imu_transmit_data_)
            sizeExpected += 2 * 3;
        if (IMU_TRANSMIT_QUATERNION_OUTPUT & imu_transmit_data_)
            sizeExpected += 2 * 4;
        if (IMU_TRANSMIT_EULER_OUTPUT & imu_transmit_data_)
            sizeExpected += 2 * 3;
        if (IMU_TRANSMIT_LINEAR_ACCELL_OUTPUT & imu_transmit_data_)
            sizeExpected += 2 * 3;
        if (IMU_TRANSMIT_PRESSURE_OUTPUT & imu_transmit_data_)
            sizeExpected += 2;
        if (IMU_TRANSMIT_ALTITUDE_OUTPUT & imu_transmit_data_)
            sizeExpected += 2;
        if (IMU_TRANSMIT_TEMP_OUTPUT & imu_transmit_data_)
            sizeExpected += 2;
        if (imu_data_double_precision_mode_)
            sizeExpected *= 2; // all data above this line is 32 bits instead of 16 bits
        sizeExpected += 4;     // have a timestamp, 4 bytes becose 32 bits integer.

        if (size != sizeExpected)
        {
            debug("Error: data packet size different of required! Ignoring data. Expected %d and receivend %d ...", sizeExpected, size);
            print_buffer();
        }

        int i = 7;

        imu_packet_data.timestamp = get_32bits_integer_from(i); // get timestamp

        i += 4; // 32 bits data

        if (imu_data_double_precision_mode_)
        {
            if (IMU_TRANSMIT_ACCEL_RAW_OUTPUT & imu_transmit_data_)
            {
                i += get_float_vector(imu_packet_data.accel_raw, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_ACCEL_OUTPUT & imu_transmit_data_)
            {
                i += get_float_vector(imu_packet_data.accel, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_GYRO_RAW_OUTPUT & imu_transmit_data_)
            {
                i += get_float_vector(imu_packet_data.gyro_raw, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_GYRO_BIAS_OUTPUT & imu_transmit_data_)
            {
                i += get_float_vector(imu_packet_data.gyro, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_GYRO_OUTPUT & imu_transmit_data_)
            {
                i += get_float_vector(imu_packet_data.gyro_bias, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_MAG_RAW_OUTPUT & imu_transmit_data_)
            {
                i += get_float_vector(imu_packet_data.mag_raw, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_MAG_OUTPUT & imu_transmit_data_)
            {
                i += get_float_vector(imu_packet_data.mag, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_ANG_VEL_OUTPUT & imu_transmit_data_)
            {
                i += get_float_vector(imu_packet_data.angular_vel, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_QUATERNION_OUTPUT & imu_transmit_data_)
            {
                i += get_float_vector(imu_packet_data.quaternion, i, 4);
            } // 4 x 4 bytes
            if (IMU_TRANSMIT_EULER_OUTPUT & imu_transmit_data_)
            {
                i += get_float_vector(imu_packet_data.euler, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_LINEAR_ACCELL_OUTPUT & imu_transmit_data_)
            {
                i += get_float_vector(imu_packet_data.linear_accel, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_PRESSURE_OUTPUT & imu_transmit_data_)
            {
                i += get_float_vector(&imu_packet_data.pressure, i, 1);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_ALTITUDE_OUTPUT & imu_transmit_data_)
            {
                i += get_float_vector(&imu_packet_data.altitude, i, 1);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_TEMP_OUTPUT & imu_transmit_data_)
            {
                i += get_float_vector(&imu_packet_data.temperature, i, 1);
            } // 3 x 4 bytes
        }
        else
        {
            if (IMU_TRANSMIT_ACCEL_RAW_OUTPUT & imu_transmit_data_)
            {
                i += get_int16_vector(imu_packet_data.accel_raw, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_ACCEL_OUTPUT & imu_transmit_data_)
            {
                i += get_int16_vector(imu_packet_data.accel, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_GYRO_RAW_OUTPUT & imu_transmit_data_)
            {
                i += get_int16_vector(imu_packet_data.gyro_raw, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_GYRO_BIAS_OUTPUT & imu_transmit_data_)
            {
                i += get_int16_vector(imu_packet_data.gyro, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_GYRO_OUTPUT & imu_transmit_data_)
            {
                i += get_int16_vector(imu_packet_data.gyro_bias, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_MAG_RAW_OUTPUT & imu_transmit_data_)
            {
                i += get_int16_vector(imu_packet_data.mag_raw, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_MAG_OUTPUT & imu_transmit_data_)
            {
                i += get_int16_vector(imu_packet_data.mag, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_ANG_VEL_OUTPUT & imu_transmit_data_)
            {
                i += get_int16_vector(imu_packet_data.angular_vel, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_QUATERNION_OUTPUT & imu_transmit_data_)
            {
                i += get_int16_vector(imu_packet_data.quaternion, i, 4);
            } // 4 x 4 bytes
            if (IMU_TRANSMIT_EULER_OUTPUT & imu_transmit_data_)
            {
                i += get_int16_vector(imu_packet_data.euler, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_LINEAR_ACCELL_OUTPUT & imu_transmit_data_)
            {
                i += get_int16_vector(imu_packet_data.linear_accel, i, 3);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_PRESSURE_OUTPUT & imu_transmit_data_)
            {
                i += get_int16_vector(&imu_packet_data.pressure, i, 1);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_ALTITUDE_OUTPUT & imu_transmit_data_)
            {
                i += get_int16_vector(&imu_packet_data.altitude, i, 1);
            } // 3 x 4 bytes
            if (IMU_TRANSMIT_TEMP_OUTPUT & imu_transmit_data_)
            {
                i += get_int16_vector(&imu_packet_data.temperature, i, 1);
            } // 3 x 4 bytes
        }
        //            printIMUData();
        return (true);
    }

    float Inertial_measurement_unit_and_compass::factor_apply(const float v1, const float factor)
    {
        if (!imu_data_double_precision_mode_)
            return (v1 / factor);
        return (v1);
    }

    float convert_g_to_ms2(const float value_in_g)
    {
        return value_in_g * GRAVITY;
    }

    bool Inertial_measurement_unit_and_compass::read_ack(void)
    {
        imu_ack_received_ = false;
        get_response();
        parse_response();
        return (imu_ack_received_);
    }

    // return true if imuBuffer contains a valid frame. Store the final position of 1st frame detected in *final_pos.
    bool Inertial_measurement_unit_and_compass::valid_frame(int *final_pos) // return TRUE if found a valid pack to parse on buffer
    {
        int id_detected = -1;
        uint16_t pack_id = 0, /*packCommand=0,*/ packLRC = 0, pack_length = 0;
        if (imu_buffer_count_ < 11)
        {
            return (false); // incomplete packet
        }
        int pos = 0;
        int end_pos = 0;
        uint8_t datal, datah;
        if (!buf_pos(&datal, pos++))
        {
            debug("\nvalid_frame(): can't get data pos %d", pos - 1);
            return (false); // incomplete packet
        }
        if (datal != IMU_PACKET_START_BYTE)
        {
            debug("\nvalid_frame(): first byte (imu_index_buffer_out_: %d) not is a start packet byte: %02x", pos - 1, datal);
            return (false); // don't have start byte
        }
        if (!buf_pos(&datal, pos++) || !buf_pos(&datah, pos++))
        {
            return (false); // incomplete packet
        }
        pack_id = datal + datah * 256;
        if (pack_id != identifier_)
        {
            if (imu_flag_detect_id_)
            {
                id_detected = pack_id;
                debug("\nNew ID %i)", pack_id);
            }
            else
            {
                debug("\nPacket from other ID (%i != %i)", pack_id, identifier_);
                return (false);
            }
        }
        if (!buf_pos(&datal, pos++) || !buf_pos(&datah, pos++))
        {
            return (false); // incomplete packet
        }
        //            packCommand      = datal + datah * 256;
        if (!buf_pos(&datal, pos++) || !buf_pos(&datah, pos++))
        {
            return (false); // incomplete packet
        }
        pack_length = datal + datah * 256;
        end_pos = 10 + pack_length;
        if (!buf_pos(&datal, end_pos))
        {
            return (false); // incomplete packet
        }
        if (datal != IMU_PACKET_END2_BYTE)
        {
            return (false); // incomplete packet
        }
        if (!buf_pos(&datal, end_pos - 1))
        {
            return (false); // incomplete packet
        }

        if (datal != IMU_PACKET_END1_BYTE)
        {
            return (false); // incomplete packet
        }

        uint16_t calcLRC = 0;
        for (int i = 1; i < end_pos - 3; i++)
        {
            if (!buf_pos(&datal, i))
                return (false);
            calcLRC += datal;
        }
        if (!buf_pos(&datal, end_pos - 3) || !buf_pos(&datah, end_pos - 2))
            return (false);
        packLRC = datal + datah * 256;
        if (packLRC != calcLRC)
        {
            return (false);
        }
        if (final_pos != NULL)
            *final_pos = end_pos;
        if (imu_flag_detect_id_)
        {
            identifier_ = id_detected;
            debug("\nStore a new ID %i", identifier_);
            imu_flag_detect_id_ = false;
        }
        return (true);
    }

    bool Inertial_measurement_unit_and_compass::init(const int ID, const std::string com_port, const uint32_t baud_rate)
    {
        identifier_ = ID;
        if (!serial_init(com_port, compute_serial_baudrate_config(baud_rate))
        ){
            debug("\n!!!!Error occurred while opening %s\n", com_port.c_str());
            imu_serial_port_ = -1;
            return false;
        }
        set_transmit_data(IMU_TRANSMIT_ACCEL_RAW_OUTPUT | IMU_TRANSMIT_ACCEL_OUTPUT | IMU_TRANSMIT_GYRO_RAW_OUTPUT | IMU_TRANSMIT_GYRO_BIAS_OUTPUT |
                        IMU_TRANSMIT_GYRO_OUTPUT | IMU_TRANSMIT_MAG_RAW_OUTPUT | IMU_TRANSMIT_MAG_OUTPUT | IMU_TRANSMIT_ANG_VEL_OUTPUT |
                        IMU_TRANSMIT_QUATERNION_OUTPUT | IMU_TRANSMIT_EULER_OUTPUT | IMU_TRANSMIT_LINEAR_ACCELL_OUTPUT |
                        IMU_TRANSMIT_ALTITUDE_OUTPUT | IMU_TRANSMIT_TEMP_OUTPUT);

        if (!send_command(IMU_GOTO_COMMAND_MODE))
           {
           debug("\n!!!!Can't put IMU in command mode...\n");
           return false;
           }

        // Configurar a saída do giroscópio em radianos por segundo
        if (!set_gyro_out_in_radians()) 
           {
           debug("\n!!!!Error occurred while setting gyro output in radians\n");
           return false;
           }

        if (!set_accelerometer_range()) 
           {
           debug("\n!!!!Error occurred while setting accelerometer range\n");
           return false;
           }

        if (!set_filter_mode(1))
           {
           debug("\n!!!!Error occurred while define filter mode\n");
           return false;
           }

        if (!reset_alignment_offset()) {
            debug("\n!!!!Error occurred while reset alignment offset\n");
            return false;
        }


        // initializate limits for convertions (when in 16 bits mode)
        imu_factor.accel_raw[0] = 1000;    // m
        imu_factor.accel_raw[1] = 1000;    // m
        imu_factor.accel_raw[2] = 1000;    // m
        imu_factor.accel[0] = 1000;       // m
        imu_factor.accel[1] = 1000;       // m
        imu_factor.accel[2] = 1000;       // m
        imu_factor.gyro_raw[0] = 10;       // rad/s
        imu_factor.gyro_raw[1] = 10;       // rad/s
        imu_factor.gyro_raw[2] = 10;       // rad/s
        imu_factor.gyro[0] = 10;          // rad/s
        imu_factor.gyro[1] = 10;          // rad/s
        imu_factor.gyro[2] = 10;          // rad/s
        imu_factor.gyro_bias[0] = 10;      // rad/s
        imu_factor.gyro_bias[1] = 10;      // rad/s
        imu_factor.gyro_bias[2] = 10;      // rad/s
        imu_factor.mag_raw[0] = 100;       // uT
        imu_factor.mag_raw[1] = 100;       // uT
        imu_factor.mag_raw[2] = 100;       // uT
        imu_factor.mag[0] = 100;          // uT
        imu_factor.mag[1] = 100;          // uT
        imu_factor.mag[2] = 100;          // uT
        imu_factor.angular_vel[0] = 10;    // rad/s
        imu_factor.angular_vel[1] = 10;    // rad/s
        imu_factor.angular_vel[2] = 10;    // rad/s
        imu_factor.quaternion[0] = 10000; // w
        imu_factor.quaternion[1] = 10000; // x
        imu_factor.quaternion[2] = 10000; // y
        imu_factor.quaternion[3] = 10000; // z
        imu_factor.euler[0] = 100;        // rad
        imu_factor.euler[1] = 100;        // rad
        imu_factor.euler[2] = 100;        // rad
        imu_factor.linear_accel[0] = 1000; // m
        imu_factor.linear_accel[1] = 1000; // m
        imu_factor.linear_accel[2] = 1000; // m
        imu_factor.altitude = 10;         // m
        imu_factor.pressure = 100;        // kPa
        imu_factor.temperature = 100;     // °C
        return (true);
    }

    bool Inertial_measurement_unit_and_compass::imu_ok(void)
    {
        if (imu_serial_port_ < 0)
            return (false);
        return (true);
    }

    bool Inertial_measurement_unit_and_compass::stop(void)
    {
        bool ret = close(imu_serial_port_);
        imu_serial_port_ = -1;
        return ret;
    }

    bool Inertial_measurement_unit_and_compass::flush(void)
    {
        uint8_t data[1000];
        while (read(imu_serial_port_, &data, 1000) > 0)
            usleep(10);
        imu_index_buffer_in_ = imu_index_buffer_out_ = imu_buffer_count_ = 0;
        return (true);
    }

    bool Inertial_measurement_unit_and_compass::set_id(const int id_par)
    {
        identifier_ = id_par;
        return (send_command(IMU_SET_ID, id_par));
    }

    IMU_operating_mode_t Inertial_measurement_unit_and_compass::get_operating_mode(void)
    {
        return ((true == in_command_mode()) ? FLEXNAV_IMU_OPERATING_MODE_COMMAND : FLEXNAV_IMU_OPERATING_MODE_STREAM);
    }

    bool Inertial_measurement_unit_and_compass::set_operating_mode(const IMU_operating_mode_t mode)
    {
        bool ret = false;

        switch (FLEXNAV_IMU_OPERATING_MODE_STREAM)
        {
        case FLEXNAV_IMU_OPERATING_MODE_STREAM:
            ret = send_command(IMU_GOTO_STREAM_MODE);
            break;

        default: // FLEXNAV_IMU_OPERATING_MODE_COMMAND
            ret = send_command(IMU_GOTO_COMMAND_MODE);
            break;
        }

        return ret;
    }

    bool Inertial_measurement_unit_and_compass::set_stream_frequency(const uint32_t frequency)
    {
        return (send_command(IMU_SET_STREAM_FREQ, frequency));
    }

    // get the current serial port
    int Inertial_measurement_unit_and_compass::get_serial_port(void) // get current serial port number
    {
        return (imu_serial_port_);
    }

    // get a response from IMU, storing data in buffer. Parameter is timeout in x 10us
    int Inertial_measurement_unit_and_compass::get_response(const uint16_t timeout) // timeout x 10 us
    {
        uint8_t data, data_ant = 0, timeout_set = timeout;
        int count = 0;
        while (timeout_set)
        {
            if (read(imu_serial_port_, &data, 1) > 0)
            {
                debug("%02x|", data);
                buf_in(data);
                count++;
                timeout_set = timeout;
                if (data == IMU_PACKET_END2_BYTE && data_ant == IMU_PACKET_END1_BYTE && count >= 10)
                    break; // when occurs end packet sequence
                data_ant = data;
            }
            else
            {
                timeout_set--;
                usleep(10);
            }
        }
        return (count);
    }

    // send a packet or command to IMU. When a packetData is used, the packet_data_size must be > 0
    bool Inertial_measurement_unit_and_compass::send_packet(const uint16_t command, const uint16_t packet_data_size, uint8_t *packet_data, const int id_to_send) // send a command and packet to imu, returning true if success
    {
        int id_to_send_ = (id_to_send < 0) ? identifier_ : id_to_send;
        uint16_t packet_data_size_ = (packet_data == NULL) ? 0 : packet_data_size;

        int full_frame_size = packet_data_size_ + 11;
        uint8_t *full_frame = (uint8_t *)malloc(full_frame_size);
        uint16_t checkSum = 0;

        full_frame[0] = IMU_PACKET_START_BYTE;
        full_frame[1] = id_to_send_ & 0xff;
        full_frame[2] = id_to_send_ >> 8;
        full_frame[3] = command & 0xff;
        full_frame[4] = command >> 8;
        full_frame[5] = packet_data_size_ & 0xff;
        full_frame[6] = packet_data_size_ >> 8;
        for (int i = 0; i < packet_data_size_; i++)
        {
            full_frame[7 + i] = packet_data[i];
        }
        for (int i = 1; i < full_frame_size - 4; i++)
            checkSum += full_frame[i];
        full_frame[full_frame_size - 4] = checkSum & 0xff;
        full_frame[full_frame_size - 3] = checkSum >> 8;
        full_frame[full_frame_size - 2] = IMU_PACKET_END1_BYTE;
        full_frame[full_frame_size - 1] = IMU_PACKET_END2_BYTE;
        bool ret = write_serial(full_frame, full_frame_size);
        debug("\nSending to IMU:");
        for (int i = 0; i < full_frame_size; i++)
            debug("%02x|", full_frame[i]);
        debug("\n");
        free(full_frame);
        return (ret);
    }

    // send a 32 bits command.
    bool Inertial_measurement_unit_and_compass::send_command(const uint16_t command, const uint32_t value) // send a command to imu, with a 32bits parameter, returning true if success
    {
        uint32_t value_ = value;
        uint8_t valueBytes[4];

        valueBytes[0] = value_ % 0x100;
        value_ /= 0x100;
        valueBytes[1] = value_ % 0x100;
        value_ /= 0x100;
        valueBytes[2] = value_ % 0x100;
        value_ /= 0x100;
        valueBytes[3] = value_ % 0x100;
        for (int retries = 0; retries < imu_max_send_retries_; retries++)
        {
            if (send_packet(command, 4, valueBytes))
               {
               for (int timeout = 0; timeout < 100; timeout++)
                   {
                   if (read_ack()) return (true);
                   }
               debug("\nFail on send command %x value %d to IMU. No ACK received.", command, value);
               }
            flush();
        }
        return (false);
    }

    // send a 16 bits command.
    bool Inertial_measurement_unit_and_compass::send_command(const uint16_t command) // send a command to IMU
    {
        for (int retries = 0; retries < imu_max_send_retries_; retries++)
        {
            if (send_packet(command, 0, NULL))
            {
                for (int timeout = 0; timeout < 100; timeout++)
                {
                    if (read_ack())
                        return (true);
                }
            }
            flush();
        }
        return (false);
    }

    // debug the buffer in HEX position
    void Inertial_measurement_unit_and_compass::print_buffer(void)
    {
        uint8_t data;
        debug("\nBUFFER: ");
        for (int i = 0; i < imu_buffer_count_; i++)
        {
            if (!buf_pos(&data, i))
                debug("[--]");
            else
                debug("[%02X]", data);
        }
    }

    // return TRUE if detected IMU in command mode.
    bool Inertial_measurement_unit_and_compass::in_command_mode(void)
    {
        return (imu_command_mode_ != 0);
    }

    // set the transmit data flag, choosing the transmit IMU data.
    bool Inertial_measurement_unit_and_compass::set_transmit_data(const uint32_t transmitDataFlag)
    {
        imu_transmit_data_ |= transmitDataFlag;
        return (send_command(IMU_SET_TRANSMIT_DATA, imu_transmit_data_));
    }

    // unset the transmit data flag, disabling the transmit IMU data.
    bool Inertial_measurement_unit_and_compass::unset_transmit_data(const uint32_t transmitDataFlag)
    {
        imu_transmit_data_ &= ~transmitDataFlag;
        return (send_command(IMU_SET_TRANSMIT_DATA, imu_transmit_data_));
    }

    // print the transmit data configuration
    /*
    void Inertial_measurement_unit_and_compass::print_IMU_transmit_data(FILE *fOutput)
    {
        fprintf(fOutput, "\nACCEL_RAW_OUTPUT    :");
        if (IMU_TRANSMIT_ACCEL_RAW_OUTPUT & imu_transmit_data_)
            fprintf(fOutput, "ENABLED");
        else
            fprintf(fOutput, "disable");
        fprintf(fOutput, "\nACCEL_OUTPUT        :");
        if (IMU_TRANSMIT_ACCEL_OUTPUT & imu_transmit_data_)
            fprintf(fOutput, "ENABLED");
        else
            fprintf(fOutput, "disable");
        fprintf(fOutput, "\nGYRO_RAW_OUTPUT     :");
        if (IMU_TRANSMIT_GYRO_RAW_OUTPUT & imu_transmit_data_)
            fprintf(fOutput, "ENABLED");
        else
            fprintf(fOutput, "disable");
        fprintf(fOutput, "\nGYRO_BIAS_OUTPUT    :");
        if (IMU_TRANSMIT_GYRO_BIAS_OUTPUT & imu_transmit_data_)
            fprintf(fOutput, "ENABLED");
        else
            fprintf(fOutput, "disable");
        fprintf(fOutput, "\nGYRO_OUTPUT         :");
        if (IMU_TRANSMIT_GYRO_OUTPUT & imu_transmit_data_)
            fprintf(fOutput, "ENABLED");
        else
            fprintf(fOutput, "disable");
        fprintf(fOutput, "\nMAG_RAW_OUTPUT      :");
        if (IMU_TRANSMIT_MAG_RAW_OUTPUT & imu_transmit_data_)
            fprintf(fOutput, "ENABLED");
        else
            fprintf(fOutput, "disable");
        fprintf(fOutput, "\nMAG_OUTPUT          :");
        if (IMU_TRANSMIT_MAG_OUTPUT & imu_transmit_data_)
            fprintf(fOutput, "ENABLED");
        else
            fprintf(fOutput, "disable");
        fprintf(fOutput, "\nANG_VEL_OUTPUT      :");
        if (IMU_TRANSMIT_ANG_VEL_OUTPUT & imu_transmit_data_)
            fprintf(fOutput, "ENABLED");
        else
            fprintf(fOutput, "disable");
        fprintf(fOutput, "\nQUATERNION_OUTPUT   :");
        if (IMU_TRANSMIT_QUATERNION_OUTPUT & imu_transmit_data_)
            fprintf(fOutput, "ENABLED");
        else
            fprintf(fOutput, "disable");
        fprintf(fOutput, "\nEULER_OUTPUT        :");
        if (IMU_TRANSMIT_EULER_OUTPUT & imu_transmit_data_)
            fprintf(fOutput, "ENABLED");
        else
            fprintf(fOutput, "disable");
        fprintf(fOutput, "\nLINEAR_ACCELL_OUTPUT:");
        if (IMU_TRANSMIT_LINEAR_ACCELL_OUTPUT & imu_transmit_data_)
            fprintf(fOutput, "ENABLED");
        else
            fprintf(fOutput, "disable");
        fprintf(fOutput, "\nPRESSURE_OUTPUT     :");
        if (IMU_TRANSMIT_PRESSURE_OUTPUT & imu_transmit_data_)
            fprintf(fOutput, "ENABLED");
        else
            fprintf(fOutput, "disable");
        fprintf(fOutput, "\nALTITUDE_OUTPUT     :");
        if (IMU_TRANSMIT_ALTITUDE_OUTPUT & imu_transmit_data_)
            fprintf(fOutput, "ENABLED");
        else
            fprintf(fOutput, "disable");
        fprintf(fOutput, "\nTEMP_OUTPUT         :");
        if (IMU_TRANSMIT_TEMP_OUTPUT & imu_transmit_data_)
            fprintf(fOutput, "ENABLED");
        else
            fprintf(fOutput, "disable");
    }
    */

    bool Inertial_measurement_unit_and_compass::set_32bits_precision(const bool precision)
    {
        if (send_command(IMU_SET_LPBUS_PRECISION, precision))
        {
            imu_data_double_precision_mode_ = precision;
            return (true);
        }
        return (false);
    }

    uint32_t Inertial_measurement_unit_and_compass::get_current_transmit_data(void) // get the current imu_transmit_data_ status
    {
        usleep(1000);
        flush();
        for (int retries = 0; retries < imu_max_send_retries_; retries++)
        {
            send_packet(IMU_GET_TRANSMIT_DATA, 0, NULL);
            if (get_response() >= 15 && parse_response())
                return (imu_transmit_data_); // RESPONSE ERROR
            flush();
        }
        return (0);
    }

    bool Inertial_measurement_unit_and_compass::get_data(void) // method to request IMU data when in command mode.
    {
        flush();
        send_packet(IMU_GET_DATA, 0, NULL);
        if (get_response() < 15)
            return (false); // RESPONSE ERROR
        if (!parse_response())
            return (false); // DON'T LOCATE RESPONSE ON INPUT BUFFER
        return (true);
    }

    bool Inertial_measurement_unit_and_compass::get_stream_loop(void) // method to run in main loop or in a task
    {
        get_response();
        if (get_buffer_size() == 0)
            return (false);
        if (parse_response())
            return (true);
        return (false);
    }

    int Inertial_measurement_unit_and_compass::detect_id(void) // method to detect ID (use if only 1 IMU is connected on tty)
    {
        imu_flag_detect_id_ = true;
        identifier_ = -1;
        flush();
        for (int idToTry = 0; idToTry < 255; idToTry++)
        {
            if (identifier_ >= 0)
                break;
            send_packet(IMU_GET_ID, 0, NULL, idToTry);
            usleep(10000);
            get_response();
            parse_response();
        }
        imu_flag_detect_id_ = false;
        return (identifier_ >= 0);
    }

    void Inertial_measurement_unit_and_compass::set_accel_raw_x_factor(const float v) { imu_factor.accel_raw[0] = v; }
    void Inertial_measurement_unit_and_compass::set_accel_raw_y_factor(const float v) { imu_factor.accel_raw[1] = v; }
    void Inertial_measurement_unit_and_compass::set_accel_raw_z_factor(const float v) { imu_factor.accel_raw[2] = v; }
    void Inertial_measurement_unit_and_compass::set_accel_x_factor(const float v) { imu_factor.accel[0] = v; }
    void Inertial_measurement_unit_and_compass::set_accel_y_factor(const float v) { imu_factor.accel[1] = v; }
    void Inertial_measurement_unit_and_compass::set_accel_z_factor(const float v) { imu_factor.accel[2] = v; }
    void Inertial_measurement_unit_and_compass::set_gyro_raw_x_factor(const float v) { imu_factor.gyro_raw[0] = v; }
    void Inertial_measurement_unit_and_compass::set_gyro_raw_y_factor(const float v) { imu_factor.gyro_raw[1] = v; }
    void Inertial_measurement_unit_and_compass::set_gyro_raw_z_factor(const float v) { imu_factor.gyro_raw[2] = v; }
    void Inertial_measurement_unit_and_compass::set_gyro_x_factor(const float v) { imu_factor.gyro[0] = v; }
    void Inertial_measurement_unit_and_compass::set_gyro_y_factor(const float v) { imu_factor.gyro[1] = v; }
    void Inertial_measurement_unit_and_compass::set_gyro_z_factor(const float v) { imu_factor.gyro[2] = v; }
    void Inertial_measurement_unit_and_compass::set_gyro_bias_x_factor(const float v) { imu_factor.gyro_bias[0] = v; }
    void Inertial_measurement_unit_and_compass::set_gyro_bias_y_factor(const float v) { imu_factor.gyro_bias[1] = v; }
    void Inertial_measurement_unit_and_compass::set_gyro_bias_z_factor(const float v) { imu_factor.gyro_bias[2] = v; }
    void Inertial_measurement_unit_and_compass::set_mag_raw_x_factor(const float v) { imu_factor.mag_raw[0] = v; }
    void Inertial_measurement_unit_and_compass::set_mag_raw_y_factor(const float v) { imu_factor.mag_raw[1] = v; }
    void Inertial_measurement_unit_and_compass::set_mag_raw_z_factor(const float v) { imu_factor.mag_raw[2] = v; }
    void Inertial_measurement_unit_and_compass::set_mag_x_factor(const float v) { imu_factor.mag[0] = v; }
    void Inertial_measurement_unit_and_compass::set_mag_y_factor(const float v) { imu_factor.mag[1] = v; }
    void Inertial_measurement_unit_and_compass::set_mag_z_factor(const float v) { imu_factor.mag[2] = v; }
    void Inertial_measurement_unit_and_compass::set_angular_vel_x_factor(const float v) { imu_factor.angular_vel[0] = v; }
    void Inertial_measurement_unit_and_compass::set_angular_vel_y_factor(const float v) { imu_factor.angular_vel[1] = v; }
    void Inertial_measurement_unit_and_compass::set_angular_vel_z_factor(const float v) { imu_factor.angular_vel[2] = v; }
    void Inertial_measurement_unit_and_compass::set_quaternion_w_factor(const float v) { imu_factor.quaternion[0] = v; }
    void Inertial_measurement_unit_and_compass::set_quaternion_x_factor(const float v) { imu_factor.quaternion[1] = v; }
    void Inertial_measurement_unit_and_compass::set_quaternion_y_factor(const float v) { imu_factor.quaternion[2] = v; }
    void Inertial_measurement_unit_and_compass::set_quaternion_z_factor(const float v) { imu_factor.quaternion[3] = v; }
    void Inertial_measurement_unit_and_compass::set_euler_x_factor(const float v) { imu_factor.euler[0] = v; }
    void Inertial_measurement_unit_and_compass::set_euler_y_factor(const float v) { imu_factor.euler[1] = v; }
    void Inertial_measurement_unit_and_compass::set_euler_z_factor(const float v) { imu_factor.euler[2] = v; }
    void Inertial_measurement_unit_and_compass::set_linear_accel_x_factor(const float v) { imu_factor.linear_accel[0] = v; }
    void Inertial_measurement_unit_and_compass::set_linear_accel_y_factor(const float v) { imu_factor.linear_accel[1] = v; }
    void Inertial_measurement_unit_and_compass::set_linear_accel_z_factor(const float v) { imu_factor.linear_accel[2] = v; }
    void Inertial_measurement_unit_and_compass::set_altitude_factor(const float v) { imu_factor.altitude = v; }
    void Inertial_measurement_unit_and_compass::set_pressure_factor(const float v) { imu_factor.pressure = v; }
    void Inertial_measurement_unit_and_compass::set_temperature_factor(const float v) { imu_factor.temperature = v; }

    uint32_t Inertial_measurement_unit_and_compass::get_error_count(void) { return (error_counter_); }         // get the buffer error count
    uint32_t Inertial_measurement_unit_and_compass::get_timestamp(void) { return (imu_packet_data.timestamp); } // return last timestamp message received from IMU.
    // float Inertial_measurement_unit_and_compass::getaccel_rawX(void) { return (factor_apply(imu_packet_data.accel_raw[0], imu_factor.accel_raw[0])); }          // return x coordinate parameter from Accelerometer Raw data
    // float Inertial_measurement_unit_and_compass::getaccel_rawY(void) { return (factor_apply(imu_packet_data.accel_raw[1], imu_factor.accel_raw[1])); }          // return y coordinate parameter from Accelerometer Raw data
    // float Inertial_measurement_unit_and_compass::getaccel_rawZ(void) { return (factor_apply(imu_packet_data.accel_raw[2], imu_factor.accel_raw[2])); }          // return z coordinate parameter from Accelerometer Raw data
    // float Inertial_measurement_unit_and_compass::getAccelX(void) { return (factor_apply(imu_packet_data.accel[0], imu_factor.accel[0])); }                   // return x coordinate parameter from Accelerometer
    // float Inertial_measurement_unit_and_compass::getAccelY(void) { return (factor_apply(imu_packet_data.accel[1], imu_factor.accel[1])); }                   // return y coordinate parameter from Accelerometer
    // float Inertial_measurement_unit_and_compass::getAccelZ(void) { return (factor_apply(imu_packet_data.accel[2], imu_factor.accel[2])); }                   // return z coordinate parameter from Accelerometer
    float Inertial_measurement_unit_and_compass::get_accel_raw_x(void)
    {
        float value_in_g = factor_apply(imu_packet_data.accel_raw[0], imu_factor.accel_raw[0]);
        return convert_g_to_ms2(value_in_g);
    }
    float Inertial_measurement_unit_and_compass::get_accel_raw_y(void)
    {
        float value_in_g = factor_apply(imu_packet_data.accel_raw[1], imu_factor.accel_raw[1]);
        return convert_g_to_ms2(value_in_g);
    }
    float Inertial_measurement_unit_and_compass::get_accel_raw_z(void)
    {
        float value_in_g = factor_apply(imu_packet_data.accel_raw[2], imu_factor.accel_raw[2]);
        return convert_g_to_ms2(value_in_g);
    }
    float Inertial_measurement_unit_and_compass::get_accel_x(void)
    {
        float value_in_g = factor_apply(imu_packet_data.accel[0], imu_factor.accel[0]);
        return convert_g_to_ms2(value_in_g);
    }
    float Inertial_measurement_unit_and_compass::get_accel_y(void)
    {
        float value_in_g = factor_apply(imu_packet_data.accel[1], imu_factor.accel[1]);
        return convert_g_to_ms2(value_in_g);
    }
    float Inertial_measurement_unit_and_compass::get_accel_z(void)
    {
        float value_in_g = factor_apply(imu_packet_data.accel[2], imu_factor.accel[2]);
        return convert_g_to_ms2(value_in_g);
    }
    float Inertial_measurement_unit_and_compass::get_gyro_raw_x(void) { return (factor_apply(imu_packet_data.gyro_raw[0], imu_factor.gyro_raw[0])); }    // return x coordinate parameter from Gyroscope Raw data
    float Inertial_measurement_unit_and_compass::get_gyro_raw_y(void) { return (factor_apply(imu_packet_data.gyro_raw[1], imu_factor.gyro_raw[1])); }    // return y coordinate parameter from Gyroscope Raw data
    float Inertial_measurement_unit_and_compass::get_gyro_raw_z(void) { return (factor_apply(imu_packet_data.gyro_raw[2], imu_factor.gyro_raw[2])); }    // return z coordinate parameter from Gyroscope Raw data
    float Inertial_measurement_unit_and_compass::get_gyro_x(void) { return (factor_apply(imu_packet_data.gyro[0], imu_factor.gyro[0])); }             // return x coordinate parameter from Gyroscope
    float Inertial_measurement_unit_and_compass::get_gyro_y(void) { return (factor_apply(imu_packet_data.gyro[1], imu_factor.gyro[1])); }             // return y coordinate parameter from Gyroscope
    float Inertial_measurement_unit_and_compass::get_gyro_z(void) { return (factor_apply(imu_packet_data.gyro[2], imu_factor.gyro[2])); }             // return z coordinate parameter from Gyroscope
    float Inertial_measurement_unit_and_compass::get_gyro_bias_x(void) { return (factor_apply(imu_packet_data.gyro_bias[0], imu_factor.gyro_bias[0])); } // return x coordinate parameter from Gyroscope Bias
    float Inertial_measurement_unit_and_compass::get_gyro_bias_y(void) { return (factor_apply(imu_packet_data.gyro_bias[1], imu_factor.gyro_bias[1])); } // return y coordinate parameter from Gyroscope Bias
    float Inertial_measurement_unit_and_compass::get_gyro_bias_z(void) { return (factor_apply(imu_packet_data.gyro_bias[2], imu_factor.gyro_bias[2])); } // return z coordinate parameter from Gyroscope Bias
    float Inertial_measurement_unit_and_compass::get_mag_raw_x(void) { return (factor_apply(imu_packet_data.mag_raw[0], imu_factor.mag_raw[0])); }       // return x coordinate parameter from Magnetometer Raw data
    float Inertial_measurement_unit_and_compass::get_mag_raw_y(void) { return (factor_apply(imu_packet_data.mag_raw[1], imu_factor.mag_raw[1])); }       // return y coordinate parameter from Magnetometer Raw data
    float Inertial_measurement_unit_and_compass::get_mag_raw_z(void) { return (factor_apply(imu_packet_data.mag_raw[2], imu_factor.mag_raw[2])); }       // return z coordinate parameter from Magnetometer Raw data
    float Inertial_measurement_unit_and_compass::get_mag_x(void) { return (factor_apply(imu_packet_data.mag[0], imu_factor.mag[0])); }                // return x coordinate parameter from Magnetometer
    float Inertial_measurement_unit_and_compass::get_mag_y(void) { return (factor_apply(imu_packet_data.mag[1], imu_factor.mag[1])); }                // return y coordinate parameter from Magnetometer
    float Inertial_measurement_unit_and_compass::get_mag_z(void) { return (factor_apply(imu_packet_data.mag[2], imu_factor.mag[2])); }                // return z coordinate parameter from Magnetometer
    float Inertial_measurement_unit_and_compass::get_angular_vel_x(void) { return (factor_apply(imu_packet_data.angular_vel[0], imu_factor.angular_vel[0])); }    // return x coordinate parameter from Angular Vel
    float Inertial_measurement_unit_and_compass::get_angular_vel_y(void) { return (factor_apply(imu_packet_data.angular_vel[1], imu_factor.angular_vel[1])); }    // return y coordinate parameter from Angular Vel
    float Inertial_measurement_unit_and_compass::get_angular_vel_z(void) { return (factor_apply(imu_packet_data.angular_vel[2], imu_factor.angular_vel[2])); }    // return z coordinate parameter from Angular Vel
    float Inertial_measurement_unit_and_compass::get_quaternion_w(void) { return (factor_apply(imu_packet_data.quaternion[0], imu_factor.quaternion[0])); } // return w coordinate parameter from Quaternion
    float Inertial_measurement_unit_and_compass::get_quaternion_x(void) { return (factor_apply(imu_packet_data.quaternion[1], imu_factor.quaternion[1])); } // return x coordinate parameter from Quaternion
    float Inertial_measurement_unit_and_compass::get_quaternion_y(void) { return (factor_apply(imu_packet_data.quaternion[2], imu_factor.quaternion[2])); } // return y coordinate parameter from Quaternion
    float Inertial_measurement_unit_and_compass::get_quaternion_z(void) { return (factor_apply(imu_packet_data.quaternion[3], imu_factor.quaternion[3])); } // return z coordinate parameter from Quaternion
    float Inertial_measurement_unit_and_compass::get_euler_x(void) { return (factor_apply(imu_packet_data.euler[0], imu_factor.euler[0])); }                // return x coordinate parameter from Euler
    float Inertial_measurement_unit_and_compass::get_euler_y(void) { return (factor_apply(imu_packet_data.euler[1], imu_factor.euler[1])); }                // return y coordinate parameter from Euler
    float Inertial_measurement_unit_and_compass::get_euler_z(void) { return (factor_apply(imu_packet_data.euler[2], imu_factor.euler[2])); }                // return z coordinate parameter from Euler
    // float Inertial_measurement_unit_and_compass::getlinear_accelX(void) { return (factor_apply(imu_packet_data.linear_accel[0], imu_factor.linear_accel[0])); } // return x coordinate parameter from linear_accel
    // float Inertial_measurement_unit_and_compass::getlinear_accelY(void) { return (factor_apply(imu_packet_data.linear_accel[1], imu_factor.linear_accel[1])); } // return y coordinate parameter from linear_accel
    // float Inertial_measurement_unit_and_compass::getlinear_accelZ(void) { return (factor_apply(imu_packet_data.linear_accel[2], imu_factor.linear_accel[2])); } // return z coordinate parameter from linear_accel
    float Inertial_measurement_unit_and_compass::get_linear_accel_x(void)
    {
        float value_in_g = factor_apply(imu_packet_data.linear_accel[0], imu_factor.linear_accel[0]);
        return convert_g_to_ms2(value_in_g);
    }
    float Inertial_measurement_unit_and_compass::get_linear_accel_y(void)
    {
        float value_in_g = factor_apply(imu_packet_data.linear_accel[1], imu_factor.linear_accel[1]);
        return convert_g_to_ms2(value_in_g);
    }
    float Inertial_measurement_unit_and_compass::get_linear_accel_z(void)
    {
        float value_in_g = factor_apply(imu_packet_data.linear_accel[2], imu_factor.linear_accel[2]);
        return convert_g_to_ms2(value_in_g);
    }
    float Inertial_measurement_unit_and_compass::get_altitude(void) { return (factor_apply(imu_packet_data.altitude, imu_factor.altitude)); }          // return Altitude
    float Inertial_measurement_unit_and_compass::get_pressure(void) { return (factor_apply(imu_packet_data.pressure, imu_factor.pressure)); }          // return Pressure
    float Inertial_measurement_unit_and_compass::get_temperature(void) { return (factor_apply(imu_packet_data.temperature, imu_factor.temperature)); } // return Temperature

    speed_t Inertial_measurement_unit_and_compass::compute_serial_baudrate_config(const unsigned int baudrate)
    {
        speed_t baudrate_;

        switch (baudrate)
        {
        case 9600:
            baudrate_ = B9600;
            break;
        case 19200:
            baudrate_ = B19200;
            break;
        case 38400:
            baudrate_ = B38400;
            break;
        case 57600:
            baudrate_ = B57600;
            break;
        case 115200:
            baudrate_ = B115200;
            break;
        case 230400:
            baudrate_ = B230400;
            break;
        case 460800:
            baudrate_ = B460800;
            break;
        case 500000:
            baudrate_ = B500000;
            break;
        case 921600:
            baudrate_ = B921600;
            break;
        case 1000000:
            baudrate_ = B1000000;
            break;
        default:
            std::cerr << "Unsupported baudrate_ rate: " << baudrate << std::endl;

            std::cout << "Using default value: " << std::to_string(115200) << std::endl;
            baudrate_ = B115200;
            break;
        }

        return baudrate_;
    }

}

#endif
