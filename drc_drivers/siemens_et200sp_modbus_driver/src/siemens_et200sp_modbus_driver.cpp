#define USE_TIMEOUT

#include "siemens_et200sp_modbus_driver.hpp"

Siemens_ET200SP_Modbus_Driver::Siemens_ET200SP_Modbus_Driver(void)
{
}

/* connect to PLC. Return true if ok, or false on fail.
   @param ip_address string (char array) to an valid IP of server
   @param port is a valid port of server
*/
bool Siemens_ET200SP_Modbus_Driver::connect(const char *ip_address, uint16_t port, uint16_t timeout)
{
    mb = modbus_new_tcp(ip_address, port);
    modbus_set_response_timeout(mb, 1, timeout);
    int error_code = modbus_connect(mb);
    if (error_code)
    {
        printf("\nCan't connect with server ... %i:", error_code);
        printf("%s\n", modbus_strerror(errno));
        modbus_close(mb);
        modbus_free(mb);
        return false;
    }
#ifdef USE_TIMEOUT
    modbus_set_response_timeout(mb, 0, timeout);
#endif
    consecutive_error_count = 0;
    return (true);
}

// stop connection.
bool Siemens_ET200SP_Modbus_Driver::disconnect(void)
{
    modbus_close(mb);
    modbus_free(mb);
    return (true);
}

// return true if PLC is is_connected
bool Siemens_ET200SP_Modbus_Driver::is_connected(void)
{
    if (consecutive_error_count >= Siemens_ET200SP_Modbus_Driver::MAXIMUM_ERROR_COUNT || mb == nullptr)
        return (false);

    int socket = modbus_get_socket(mb);

    return socket >= 0;
}

// read all data (PLC -> PC). Return false on fail.
int Siemens_ET200SP_Modbus_Driver::read(void)
{
    elapsed_time = get_system_timestamp_us();
    int modbus_result = modbus_read_registers(mb, __initialRegisterForRead, sizeof(read_registers)/2, (uint16_t *)&read_registers);
    if (modbus_result <= 0)
    {
        error_count++;
        consecutive_error_count++;
    }
    else
    {
        consecutive_error_count = 0;
    }
    last_read_write_time = get_system_timestamp_us();
    elapsed_time = last_read_write_time - elapsed_time;
    last_read_write_time = elapsed_time;
    return (modbus_result);
}

// write all data (PC -> PLC). Return false on fail.
int Siemens_ET200SP_Modbus_Driver::write(void)
{
    elapsed_time = get_system_timestamp_us();
    int modbus_result = modbus_write_registers(mb, __initialRegisterForWrite, sizeof(write_registers)/2, (uint16_t *)&write_registers);
    if (modbus_result <= 0)
    {
        error_count++;
        consecutive_error_count++;
    }
    else
    {
        consecutive_error_count = 0;
    }
    last_read_write_time = get_system_timestamp_us();
    elapsed_time = last_read_write_time - elapsed_time;
    last_read_write_time = elapsed_time;
    return modbus_result;
}

// write all data (PC -> PLC). Return false on fail.
int Siemens_ET200SP_Modbus_Driver::syncro(void)
{
    elapsed_time = get_system_timestamp_us();
    int modbus_result = modbus_write_and_read_registers(mb, __initialRegisterForWrite, sizeof(write_registers)/2, (uint16_t *)&write_registers,
                                                  __initialRegisterForRead, sizeof(read_registers)/2, (uint16_t *)&read_registers);
    if (modbus_result <= 0)
    {
        error_count++;
        consecutive_error_count++;
    }
    else
    {
        consecutive_error_count = 0;
    }
    last_read_write_time = get_system_timestamp_us();
    elapsed_time = last_read_write_time - elapsed_time;
    last_read_write_time = elapsed_time;
    return (modbus_result);
}

// get encoder value (raw data 32 bits)
int32_t Siemens_ET200SP_Modbus_Driver::get_left_encoder_value(void)
{
    return ((int32_t)(((uint32_t)read_registers.left_encoder_high << 16) | read_registers.left_encoder_low));
}

// get encoder value (raw data 32 bits)
int32_t Siemens_ET200SP_Modbus_Driver::get_right_encoder_value(void)
{
    return ((int32_t)(((uint32_t)read_registers.right_encoder_high << 16) | read_registers.right_encoder_low));
}

int64_t Siemens_ET200SP_Modbus_Driver::get_left_encoder_counts()
{
    static int64_t encoderAnt = get_left_encoder_value();
    int64_t encoderAtu = get_left_encoder_value();
    int64_t encoderDif = encoderAtu - encoderAnt;
    if (encoderDif >=  INT32_MAX / 2) encoderDif -= (int64_t)UINT32_MAX + 1;
    if (encoderDif <= -INT32_MAX / 2) encoderDif += (int64_t)UINT32_MAX + 1;
    left_encoder_counter += encoderDif;
    encoderAnt = encoderAtu;
    return(left_encoder_counter);
}

int64_t Siemens_ET200SP_Modbus_Driver::get_right_encoder_counts()
{
    static int64_t encoderAnt = get_right_encoder_value();
    int64_t encoderAtu = get_right_encoder_value();
    int64_t encoderDif = encoderAtu - encoderAnt;
    if (encoderDif >  INT32_MAX / 2) encoderDif -= (int64_t)UINT32_MAX + 1;
    if (encoderDif < -INT32_MAX / 2) encoderDif += (int64_t)UINT32_MAX + 1;
    right_encoder_counter += encoderDif;
    encoderAnt = encoderAtu;
    return(right_encoder_counter);
}

void Siemens_ET200SP_Modbus_Driver::set_left_encoder_counts(int64_t value)
{
    left_encoder_counter = value;
}

void Siemens_ET200SP_Modbus_Driver::set_right_encoder_counts(int64_t value)
{
    right_encoder_counter = value;
}

// set timestamp value
void Siemens_ET200SP_Modbus_Driver::set_message_timestamp_ms(uint64_t value)
{
  timestamp_value = value;  
}


// get timestamp value 
uint64_t Siemens_ET200SP_Modbus_Driver::get_message_timestamp_ms(void)
{
    static int32_t timestamp_ant = get_raw_timestamp_ms();
    int32_t timestamp_atu = get_raw_timestamp_ms();
    int32_t timestamp_dif = 0;
    if (timestamp_atu >= timestamp_ant)
       {
       timestamp_dif = timestamp_atu - timestamp_ant;
       timestamp_value += timestamp_dif;
       }
    else
       {
       timestamp_dif = INT32_MAX - timestamp_ant;
       timestamp_dif += timestamp_atu;
       timestamp_value += timestamp_dif;
       }
    timestamp_ant = timestamp_atu;
    return(timestamp_value);
}


// get timestamp value in milliseconds
uint32_t Siemens_ET200SP_Modbus_Driver::get_raw_timestamp_ms(void)
{
    return (((uint32_t)(read_registers.timestamp_high << 16) | read_registers.timestamp_low));
}

// get joystick value
int8_t Siemens_ET200SP_Modbus_Driver::get_joystick_up(void)
{
    return (read_registers.joy_up);
}

// get joystick value
int8_t Siemens_ET200SP_Modbus_Driver::get_joystick_down(void)
{
    return (read_registers.joy_down);
}

// get joystick value
int8_t Siemens_ET200SP_Modbus_Driver::get_joystick_left(void)
{
    return (read_registers.joy_left);
}

// get joystick value
int8_t Siemens_ET200SP_Modbus_Driver::get_joystick_right(void)
{
    return (read_registers.joy_right);
}


// return true if some obstacle detected in front sensor
uint8_t Siemens_ET200SP_Modbus_Driver::get_front_laser_scanner_active_zone(void)
{
    uint8_t active_zone = 0;
    if (read_registers.d_inputs & ROBOT_LS_FRONT_BIT0) active_zone |= 0b0001;
    if (read_registers.d_inputs & ROBOT_LS_FRONT_BIT1) active_zone |= 0b0010;
    if (read_registers.d_inputs & ROBOT_LS_FRONT_BIT2) active_zone |= 0b0100;
    if (read_registers.d_inputs & ROBOT_LS_FRONT_BIT3) active_zone |= 0b1000;
    return (active_zone);
}

// return true if some obstacle detected in rear sensor
uint8_t Siemens_ET200SP_Modbus_Driver::get_rear_laser_scanner_active_zone(void)
{
    uint8_t active_zone = 0;
    if (read_registers.d_inputs & ROBOT_LS_REAR_BIT0) active_zone |= 0b0001;
    if (read_registers.d_inputs & ROBOT_LS_REAR_BIT1) active_zone |= 0b0010;
    if (read_registers.d_inputs & ROBOT_LS_REAR_BIT2) active_zone |= 0b0100;
    if (read_registers.d_inputs & ROBOT_LS_REAR_BIT3) active_zone |= 0b1000;
    return (active_zone);
}

bool Siemens_ET200SP_Modbus_Driver::has_error_code(void)
{
    return (read_registers.error_code > 0);
}


// return true if no error code
bool Siemens_ET200SP_Modbus_Driver::has_no_error_code(void)
{
    return (read_registers.error_code == 0);
}

// return true if a reset occured
bool Siemens_ET200SP_Modbus_Driver::has_initial_error_code(void)
{
    return (read_registers.error_code & ROBOT_ERROR_INITIAL_STATUS) > 0;
}

// return true if detect a error on left motor
bool Siemens_ET200SP_Modbus_Driver::has_left_motor_error(void)
{
    return (read_registers.error_code & ROBOT_ERROR_LEFT_MOTOR) > 0;
}

// return true if detect a error on right motor
bool Siemens_ET200SP_Modbus_Driver::has_right_motor_error(void)
{
    return (read_registers.error_code & ROBOT_ERROR_RIGHT_MOTOR) > 0;
}

// return true if detect a error on table motor
bool Siemens_ET200SP_Modbus_Driver::has_table_motor_error(void)
{
    return (read_registers.error_code & ROBOT_ERROR_TABLE_MOTOR) > 0;
}

// return true if detect a error while loading (table)
bool Siemens_ET200SP_Modbus_Driver::has_loading_error(void)
{
    return (read_registers.error_code & ROBOT_ERROR_LOAD) > 0;
}

// return true if detect a error while unloading (table)
bool Siemens_ET200SP_Modbus_Driver::has_unloading_error(void)
{
    return (read_registers.error_code & ROBOT_ERROR_UNLOAD) > 0;
}

// return true if detect a battery or energy error
bool Siemens_ET200SP_Modbus_Driver::has_battery_error(void)
{
    return (read_registers.error_code & ROBOT_ERROR_BATTERY) > 0;
}

// return true if detect a docking error
bool Siemens_ET200SP_Modbus_Driver::has_docking_error(void)
{
    return (read_registers.error_code & ROBOT_ERROR_DOCKING) > 0;
}

// return true if waiting for PRESS RESET to reset errorcode
bool Siemens_ET200SP_Modbus_Driver::is_waiting_user_reset_error_codes(void)
{
    return (read_registers.error_code & ROBOT_ERROR_WAIT_FOR_RESET) > 0;
}

// return true if detect an undefined error
bool Siemens_ET200SP_Modbus_Driver::has_undefined_error(void)
{
    return (read_registers.error_code & ROBOT_ERROR_UNDEFINED) > 0;
}

// return true if manual/auto switch is in auto position
bool Siemens_ET200SP_Modbus_Driver::is_automatic_mode_selected(void)
{
    return (read_registers.d_inputs & ROBOT_STATUS_AUTO_SW) > 0;
}

// return true if manual/auto switch is in manual position
bool Siemens_ET200SP_Modbus_Driver::is_manual_mode_selected(void)
{
    return (read_registers.d_inputs & ROBOT_STATUS_MAN_SW) > 0;
}

// return true if reset button is pressed
bool Siemens_ET200SP_Modbus_Driver::is_reset_button_pressed(void)
{
    return (read_registers.d_inputs & ROBOT_STATUS_RESET_SW) > 0;
}

// return front load sensor status (true - false)
bool Siemens_ET200SP_Modbus_Driver::get_front_load_sensor_state(void)
{
    return (read_registers.d_inputs & ROBOT_STATUS_FRONT_LOAD_SENSOR) > 0;
}

// return rear load sensor status (true - false)
bool Siemens_ET200SP_Modbus_Driver::get_rear_load_sensor_state(void)
{
    return (read_registers.d_inputs & ROBOT_STATUS_REAR_LOAD_SENSOR) > 0;
}


// return true if joystick is is_connected
bool Siemens_ET200SP_Modbus_Driver::is_joystick_connected(void)
{
    return (read_registers.d_inputs & ROBOT_STATUS_JOYSTICK_CONNECTED) > 0;
}

// return status of LOAD flag
bool Siemens_ET200SP_Modbus_Driver::is_loaded(void)
{
    return (read_registers.d_inputs & ROBOT_STATUS_LOADED) > 0;
}

// return true if charging
bool Siemens_ET200SP_Modbus_Driver::is_charging(void)
{
    return (read_registers.d_inputs & ROBOT_STATUS_CHARGING) > 0;
}

// return true if safetty is ok, and low level robot is operacional
bool Siemens_ET200SP_Modbus_Driver::is_operational(void)
{
    return !is_emergency_lock();
}

// return true if in maintance mode
bool Siemens_ET200SP_Modbus_Driver::is_maintenance_mode(void)
{
    return ((read_registers.error_code & ROBOT_ERROR_MAINTANCE_MODE) > 0);
}

// return true on safetty lock by an is_emergency_lock signal
bool Siemens_ET200SP_Modbus_Driver::is_emergency_lock()
{
    return !((read_registers.error_code & ROBOT_ERROR_EMERGENCY) > 0);
}

// set the robot signaling lights
void Siemens_ET200SP_Modbus_Driver::set_front_red_light_state(robot_signaling_status_t state)
{
    write_registers.front_red_lights = state;
}

// set the robot signaling lights
void Siemens_ET200SP_Modbus_Driver::set_rear_red_light_state(robot_signaling_status_t state)
{
    write_registers.rear_red_lights = state;
}

// set the robot signaling lights
void Siemens_ET200SP_Modbus_Driver::set_front_green_light_state(robot_signaling_status_t state)
{
    write_registers.front_green_lights = state;
}

// set the robot signaling lights
void Siemens_ET200SP_Modbus_Driver::set_rear_green_light_state(robot_signaling_status_t state)
{
    write_registers.rear_green_lights = state;
}

// set the robot signaling lights
void Siemens_ET200SP_Modbus_Driver::set_left_light_state(robot_signaling_status_t state)
{
    write_registers.left_yellow_lights = state;
}

// set the robot signaling lights
void Siemens_ET200SP_Modbus_Driver::set_right_light_state(robot_signaling_status_t state)
{
    write_registers.right_yellow_lights = state;
}

// set the robot signaling lights
void Siemens_ET200SP_Modbus_Driver::set_buzzer_state(robot_signaling_status_t state)
{
    write_registers.horn_buzzer = state;
}

// set load/unload timeout
void Siemens_ET200SP_Modbus_Driver::set_load_unload_timeout_seconds(uint8_t seconds)
{
    write_registers.load_unload_timeout_seconds = seconds;
}

// set load command
void Siemens_ET200SP_Modbus_Driver::send_command(robot_command_t cmd)
{
    write_registers.command = cmd;
}

// set scanner config
void Siemens_ET200SP_Modbus_Driver::set_laser_scanner_config(uint8_t preset)
{
    write_registers.laser_scanner_select = preset;
}

// // set scanner config
// void Siemens_ET200SP_Modbus_Driver::setRearLaserScannerConfig(uint8_t preset)
// {
//     write_registers.rearLSSelect = preset;
// }

// get the robot signaling lights
robot_signaling_status_t Siemens_ET200SP_Modbus_Driver::get_front_red_light_state(void)
{
    return ((robot_signaling_status_t)write_registers.front_red_lights);
}

// get the robot signaling lights
robot_signaling_status_t Siemens_ET200SP_Modbus_Driver::get_rear_red_light_state(void)
{
    return ((robot_signaling_status_t)write_registers.rear_red_lights);
}

// get the robot signaling lights
robot_signaling_status_t Siemens_ET200SP_Modbus_Driver::get_front_green_light_state(void)
{
    return ((robot_signaling_status_t)write_registers.front_green_lights);
}

// get the robot signaling lights
robot_signaling_status_t Siemens_ET200SP_Modbus_Driver::get_rear_green_light_state(void)
{
    return ((robot_signaling_status_t)write_registers.rear_green_lights);
}

// get the robot signaling lights
robot_signaling_status_t Siemens_ET200SP_Modbus_Driver::get_left_light_state(void)
{
    return ((robot_signaling_status_t)write_registers.left_yellow_lights);
}

// get the robot signaling lights
robot_signaling_status_t Siemens_ET200SP_Modbus_Driver::get_right_light_state(void)
{
    return ((robot_signaling_status_t)write_registers.right_yellow_lights);
}

// get the robot signaling lights
robot_signaling_status_t Siemens_ET200SP_Modbus_Driver::get_buzzer_state(void)
{
    return ((robot_signaling_status_t)write_registers.horn_buzzer);
}

// get load/unload timeout
robot_signaling_status_t Siemens_ET200SP_Modbus_Driver::get_load_unload_timeout_seconds(void)
{
    return ((robot_signaling_status_t)write_registers.load_unload_timeout_seconds);
}

// get load command
robot_command_t Siemens_ET200SP_Modbus_Driver::get_command(void)
{
    return ((robot_command_t)read_registers.command);
}

// get scanner config
uint8_t Siemens_ET200SP_Modbus_Driver::get_laser_scanner_config(void)
{
    return (write_registers.laser_scanner_select);
}

uint64_t Siemens_ET200SP_Modbus_Driver::get_system_timestamp_us(void)
{
    auto now = std::chrono::high_resolution_clock::now();
    uint64_t timePoint = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
    return timePoint;
}

// get last read write operation time in microseconds
uint64_t Siemens_ET200SP_Modbus_Driver::get_last_read_write_time(void)
{
    return last_read_write_time;
}

// get last read write operation time in microseconds
uint32_t Siemens_ET200SP_Modbus_Driver::get_error_count(void)
{
    return error_count;
}

// get last mission sequence number. When new number , new mission.
uint8_t Siemens_ET200SP_Modbus_Driver::get_mission_seq(void)
{
    return (read_registers.mission_seq);
}

// get mission type
uint8_t Siemens_ET200SP_Modbus_Driver::get_mission_cmd(void) // return new mission cmd and reset new command flag
{
    return (read_registers.mission_cmd);
}

bool Siemens_ET200SP_Modbus_Driver::is_mission_cancel_cmd(void)
{
    return (get_mission_cmd() & MISSION_TYPE_CANCEL) > 0;
}

bool Siemens_ET200SP_Modbus_Driver::is_mission_move_only(void)
{
    return (get_mission_cmd() & MISSION_TYPE_MOVE) > 0;
}

bool Siemens_ET200SP_Modbus_Driver::is_mission_load_at_dest(void)
{
    return (get_mission_cmd() & MISSION_TYPE_LOAD) > 0;
}

bool Siemens_ET200SP_Modbus_Driver::is_mission_unload_at_dest(void)
{
    return (get_mission_cmd() & MISSION_TYPE_UNLOAD) > 0;
}

bool Siemens_ET200SP_Modbus_Driver::is_mission_dock_from_rear(void)
{
    return (get_mission_cmd() & MISSION_TYPE_FROM_REAR) > 0;
}

bool Siemens_ET200SP_Modbus_Driver::is_new_mission_cmd(void) // return true when detection of new sequence (new command flag)
{
    static uint8_t last_mission_seq = get_mission_seq();
    uint8_t current_mission_seq = get_mission_seq();
    if (current_mission_seq != last_mission_seq)
    {
        last_mission_seq = current_mission_seq;
        return true; // new mission command detected
    }
    return false; // no new mission command
}

  // get mission node
void Siemens_ET200SP_Modbus_Driver::get_mission_node_label(char * label) // return the node label dest for mission
{
    if (label == nullptr) return; // avoid null pointer
    label[0] = read_registers.mission_node[1];
    label[1] = read_registers.mission_node[0];
    label[2] = read_registers.mission_node[3];
    label[3] = read_registers.mission_node[2];
    label[4] = '\0'; // ensure null termination
}

  // set mission status / feedback and auto increase sequence number
void Siemens_ET200SP_Modbus_Driver::set_mission_status(mission_status_tp status)
{
    write_registers.mission_status = status;
    write_registers.mission_seq++; // increase sequence number
}

  // send a text message
void Siemens_ET200SP_Modbus_Driver::send_message(char * msg /*must be a 20bytes string*/, message_tp msgtp, uint8_t msgCode)
  {
    if (msg == nullptr) return; // avoid null pointer
    write_registers.message_type = msgtp;
    write_registers.message_code = msgCode;
    for (int i = 0; i < 20; i++)
    {
        int j = i ^ 1;
        if (msg[i] != '\0')
            write_registers.message_value[j] = msg[i];
        else
            write_registers.message_value[j] = 0; // ensure null termination
    }
  }
  // \NEW 
