#ifndef plc_modbus_lib
#define plc_modbus_lib

#define _USE_MATH_DEFINES
#include <chrono>
#include <cmath>
#include <errno.h>
#include <modbus/modbus.h>
#include <netinet/in.h>
#include <netinet/tcp.h> // To allow TCP_NODELAY
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define MILLICSEC2RPM 60000.0

typedef enum {
  SIGNALING_OFF = 0,
  SIGNALING_ON = 1,
  SIGNALING_FAST_BLINK = 2,
  SIGNALING_BLINK = 3,
  SIGNALING_SLOW_BLINK = 4,
  SIGNALING_BY_PLC = 255
} robot_signaling_status_t;

typedef enum {
  ROBOT_STOP_LOADING = 0,
  ROBOT_LOADING_FROM_FRONT = 1,
  ROBOT_LOADING_FROM_REAR = 2,
  ROBOT_UNLOADING_FROM_FRONT = 3,
  ROBOT_UNLOADING_FROM_REAR = 4,
  ROBOT_SET_LOAD_ON = 16,
  ROBOT_SET_LOAD_OFF = 32,
  ROBOT_SHUTDOWN = 64, // NEW - when this command send/received, start a safe shutdown routines 
  ROBOT_STOPPING_ALL_SERVICES = 112, // NEW - signaling the shuting down is occurring on other parts
  ROBOT_RESET_ALL_ERROR_CODES = 128
} robot_command_t;

/*NEW*/

typedef enum {
     MESSAGE_INFO = 0,
     MESSAGE_WARNING = 1,
     MESSAGE_ERROR = 2,
     MESSAGE_FATAL_ERROR = 3
}    message_tp;

typedef enum {
     NO_MISSION = 0,
     MISSION_RUNNING = 1, 
     MISSION_ERROR = 2,
     MISSION_PAUSE = 3,
     MISSION_CANCEL = 4,
     MISSION_FINISH = 5
}    mission_status_tp;

typedef enum {
     MISSION_TYPE_MOVE      = (0b100 << 5),   // NEW boolean to set Move-only Mission Type
     MISSION_TYPE_LOAD      = (0b010 << 5),   // NEW boolean to set load at dest mission type 
     MISSION_TYPE_UNLOAD    = (0b001 << 5),   // NEW boolean to set unload at dest mission type
     MISSION_TYPE_FROM_REAR = (0b10),         // NEW boolean to set "dock from rear" at dest. When unset, dock from front
     MISSION_TYPE_CANCEL    = (0b01),         // NEW boolean to send a "cancel" command to mission
}    mission_command_tp;

/* \NEW */

// old size: 11 registradores 2 bytes cada : 22 bytes
// new size: 13 registradores 2 bytes cada : 26 bytes

struct read_registers_struct {
  uint16_t left_encoder_high = 0; // FIRST 16 BITS OF MB BLOCK
  uint16_t left_encoder_low = 0;  // OTHER 16 BITS OF MB BLOCK
  uint16_t right_encoder_high = 0;
  uint16_t right_encoder_low = 0;
  uint16_t timestamp_high = 0;
  uint16_t timestamp_low = 0;
  int8_t joy_down = 0; // MOST SIGNIFICANT BYTE OF WORD
  int8_t joy_up = 0;   // LOW SIGNIFICANT BYTE OF WORD
  int8_t joy_right = 0;
  int8_t joy_left = 0;
  uint16_t d_inputs = 0;
  uint16_t error_code = 0;
  uint8_t receive1 = 0;
  uint8_t command = 0;
  uint8_t mission_seq = 0; // NEW auto increasing when a new mission is created. 
  uint8_t mission_cmd = 0; // NEW 
  char   mission_node[4] = {0,0,0,0}; // NEW
} read_registers;

// old size: 6 words, 12 bytes
// new size: 18 words, 36 bytes

struct write_registers_struct {
  uint8_t rear_red_lights = 0;
  uint8_t front_red_lights = 0;

  uint8_t rear_green_lights = 0;
  uint8_t front_green_lights = 0;

  uint8_t right_yellow_lights = 0;
  uint8_t left_yellow_lights = 0;

  uint8_t load_unload_timeout_seconds = 0;
  uint8_t horn_buzzer = 0;

  uint8_t unused1 = 0;
  uint8_t laser_scanner_select = 0x95;

  uint8_t reserverd = 0;
  uint8_t command = 0;

  uint8_t mission_status = 0;   // NEW (mission_status_tp)
  uint8_t mission_seq = 0;      // NEW (sequencial number. Increase to send new values.)
  
  uint8_t message_code = 0;     // NEW code of message, to use in complex table messages
  uint8_t message_type = 0;     // NEW message type (message_tp)

  char    message_value[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // NEW space to 20 char message
} write_registers;

template <typename T, typename U>
double convert_count_to_rad(T encoder_counts, U encoder_max_counts) {

  // Convert encoder counts to angular displacement measured in the wheel
  // total counts = 40000
  double angular_position = static_cast<double>(
      ((encoder_counts) / static_cast<double>(encoder_max_counts)) * 2.0 *
      M_PI);

  return angular_position;
}

template <typename T, typename U>
double convert_count_to_delta_rad(T encoder_counts, T last_encoder_counts,
                                  U encoder_max_counts) {

  // Convert encoder counts to angular displacement measured in the wheel
  double angular_displacement =
      static_cast<double>(((encoder_counts - last_encoder_counts) /
                           static_cast<double>(encoder_max_counts)) *
                          2.0 * M_PI);

  return angular_displacement;
}

template <typename T, typename U, typename V>
double convert_count_to_rad_s(T encoder_counts, T last_encoder_counts,
                              U encoder_max_counts, V delta_t_ms) {

  // Convert encoder counts to angular displacement measured in the wheel
  double wheel_vel =
      static_cast<double>(((encoder_counts - last_encoder_counts) /
                           static_cast<double>(encoder_max_counts)) /
                          (delta_t_ms / 1000.0) * 2 * M_PI);

  return wheel_vel;
}

template <typename T, typename U, typename V>
double convert_count_to_rpm(T encoder_counts, T last_encoder_counts,
                            U encoder_max_counts, V delta_t) {

  // Convert encoder counts to rpm measured in the wheel axis
  double rpm =
      static_cast<double>((encoder_counts - last_encoder_counts) /
                          static_cast<double>(encoder_max_counts) /
                          (static_cast<double>(delta_t) / MILLICSEC2RPM));

  return rpm;
}

class Siemens_ET200SP_Modbus_Driver {

private:
  static constexpr uint16_t ROBOT_LS_FRONT_BIT0 = 0b1 << 8;
  static constexpr uint16_t ROBOT_LS_FRONT_BIT1 = 0b1 << 9;
  static constexpr uint16_t ROBOT_LS_FRONT_BIT2 = 0b1 << 10;
  static constexpr uint16_t ROBOT_LS_FRONT_BIT3 = 0b1 << 11;
  static constexpr uint16_t ROBOT_LS_REAR_BIT0 = 0b1 << 12;
  static constexpr uint16_t ROBOT_LS_REAR_BIT1 = 0b1 << 13;
  static constexpr uint16_t ROBOT_LS_REAR_BIT2 = 0b1 << 14;
  static constexpr uint16_t ROBOT_LS_REAR_BIT3 = 0b1 << 15;

  static constexpr uint16_t ROBOT_STATUS_AUTO_SW =
      0b1 << 0; //- man/aut switch on auto position
  static constexpr uint16_t ROBOT_STATUS_MAN_SW =
      0b1 << 1; //- man/aut switch on man position
  static constexpr uint16_t ROBOT_STATUS_RESET_SW =
      0b1 << 2; //- Reset push button pressed
  static constexpr uint16_t ROBOT_STATUS_FRONT_LOAD_SENSOR =
      0b1 << 3; //- Front load sensor NC (high level = not detecting)
  static constexpr uint16_t ROBOT_STATUS_REAR_LOAD_SENSOR =
      0b1 << 4; //- Rear load sensor NC (high level = not detecting)
  static constexpr uint16_t ROBOT_STATUS_JOYSTICK_CONNECTED =
      0b1 << 5; //- Joystick is_connected
  static constexpr uint16_t ROBOT_STATUS_LOADED =
      0b1 << 6; //- Loaded (transport table)
  static constexpr uint16_t ROBOT_STATUS_CHARGING = 0b1
                                                    << 7; //- Charging battery

  static constexpr uint16_t ROBOT_ERROR_NOERROR = 0b0; // No errors
  static constexpr uint16_t ROBOT_ERROR_WAIT_FOR_RESET =
      0b1 << 8; // reset button press waiting
  static constexpr uint16_t ROBOT_ERROR_UNDEFINED = 0b1 << 9; // undefined error
  static constexpr uint16_t ROBOT_ERROR_INITIAL_STATUS =
      0b1 << 10; // initial status error (reset)
  static constexpr uint16_t ROBOT_ERROR_LEFT_MOTOR = 0b1
                                                     << 11; // left motor error
  static constexpr uint16_t ROBOT_ERROR_RIGHT_MOTOR =
      0b1 << 12; // right motor error
  static constexpr uint16_t ROBOT_ERROR_TABLE_MOTOR =
      0b1 << 13; // table motor error
  static constexpr uint16_t ROBOT_ERROR_LOAD = 0b1
                                               << 14; // load error (timeout)
  static constexpr uint16_t ROBOT_ERROR_UNLOAD =
      0b1 << 15; // unload error (timeout)
  static constexpr uint16_t ROBOT_ERROR_EMERGENCY = 0b1 << 0; // EMERGENCY
  static constexpr uint16_t ROBOT_ERROR_BATTERY = 0b1 << 1;   //
  static constexpr uint16_t ROBOT_ERROR_DOCKING = 0b1 << 2;   // docking error
  static constexpr uint16_t ROBOT_ERROR_MAINTANCE_MODE = 0b1
                                                         << 3; // maintance mode

  static constexpr uint32_t MAXIMUM_ERROR_COUNT = 5;

  static constexpr uint16_t __ReadBlockSize = 11; //(* 2 bytes)
  static constexpr uint16_t __WriteBlockSize = 6; //(* 2 bytes)
  static constexpr uint16_t __initialRegisterForRead =
      0x00; // offset of datablock on PC to read registers
  static constexpr uint16_t __initialRegisterForWrite =
      0x1C; // 1C word from plc to write registers

  modbus_t *mb = nullptr;

  uint32_t error_count = 0;
  uint32_t consecutive_error_count = 0;
  uint64_t last_read_write_time = 0;
  uint64_t elapsed_time = 0;

  int64_t left_encoder_counter = 0;
  int64_t right_encoder_counter = 0;
  uint64_t timestamp_value =
      0; // PLC timestamp (32bits) absolute -> timestamp_value (64bits) relative

  uint64_t get_system_timestamp_us(void);

public:
  Siemens_ET200SP_Modbus_Driver(void);

  /* connect to PLC. Return true if ok, or false on fail.
     @param ip_addres string (char array) to an valid IP of server
     @param port is a valid port of server
  */
  bool connect(const char *ip_addres, uint16_t port, uint16_t timeout = 20000);

  // stop connection.
  bool disconnect(void);

  // return true if PLC is is_connected
  bool is_connected(void);

  // read all data (PLC -> PC). Return false on fail.
  int read(void);

  // write all data (PC -> PLC). Return false on fail.
  int write(void);

  // write all data (PC -> PLC). Return false on fail.
  int syncro(void);

  // get encoder value in counts
  int64_t get_left_encoder_counts(void);

  // get encoder value in counts
  int64_t get_right_encoder_counts(void);

  // get raw data encoder value
  int32_t get_left_encoder_value(void);

  // get raw data encoder value
  int32_t get_right_encoder_value(void);

  // set left encoder value (relative value)
  void set_left_encoder_counts(int64_t value);

  // set right encoder value (relative value)
  void set_right_encoder_counts(int64_t value);

  // get timestamp 64 bits value
  uint64_t get_message_timestamp_ms(void);

  // get timestamp raw data value (32 bits)
  uint32_t get_raw_timestamp_ms(void);

  // set timestamp 64 bits value
  void set_message_timestamp_ms(uint64_t value);

  // get joystick value
  int8_t get_joystick_up(void);

  // get joystick value
  int8_t get_joystick_down(void);

  // get joystick value
  int8_t get_joystick_left(void);

  // get joystick value
  int8_t get_joystick_right(void);

  // return 4 bits from front LS sensor
  uint8_t get_front_laser_scanner_active_zone(void);

  // return 4 bits from rear LS sensor
  uint8_t get_rear_laser_scanner_active_zone(void);

  // return true if charging
  bool is_charging(void);

  bool has_error_code(void);

  // return true if no error code
  bool has_no_error_code(void);

  // return true if a reset occured
  bool has_initial_error_code(void);

  // return true if detect a error on left motor
  bool has_left_motor_error(void);

  // return true if detect a error on right motor
  bool has_right_motor_error(void);

  // return true if detect a error on table motor
  bool has_table_motor_error(void);

  // return true if detect a error while loading (table)
  bool has_loading_error(void);

  // return true if detect a error while unloading (table)
  bool has_unloading_error(void);

  // return true if detect a battery or energy error
  bool has_battery_error(void);

  // return true if detect a docking error
  bool has_docking_error(void);

  // return true if waiting for PRESS RESET to reset errorcode
  bool is_waiting_user_reset_error_codes(void);

  // return true if detect an undefined error
  bool has_undefined_error(void);

  // return true if manual/auto switch is in auto position
  bool is_automatic_mode_selected(void);

  // return true if manual/auto switch is in manual position
  bool is_manual_mode_selected(void);

  // return true if reset button is pressed
  bool is_reset_button_pressed(void);

  // return front load sensor status (true - false)
  bool get_front_load_sensor_state(void);

  // return rear load sensor status (true - false)
  bool get_rear_load_sensor_state(void);

  // return true if joystick is is_connected
  bool is_joystick_connected(void);

  // return status of LOAD flag
  bool is_loaded(void);

  // return true if safetty is ok, and low level robot is operacional
  bool is_operational(void);

  // return true if in maintance mode
  bool is_maintenance_mode(void);

  // return true on safetty lock by an is_emergency_lock signal
  bool is_emergency_lock();

  // set the robot signaling lights
  void set_front_red_light_state(robot_signaling_status_t state);

  // set the robot signaling lights
  void set_rear_red_light_state(robot_signaling_status_t state);

  // set the robot signaling lights
  void set_front_green_light_state(robot_signaling_status_t state);

  // set the robot signaling lights
  void set_rear_green_light_state(robot_signaling_status_t state);

  // set the robot signaling lights
  void set_left_light_state(robot_signaling_status_t state);

  // set the robot signaling lights
  void set_right_light_state(robot_signaling_status_t state);

  // set the robot signaling lights
  void set_buzzer_state(robot_signaling_status_t state);

  // set load/unload timeout
  void set_load_unload_timeout_seconds(uint8_t seconds);

  // send a command command
  void send_command(robot_command_t status);

  // set scanner config
  void set_laser_scanner_config(uint8_t preset);

  // get the robot signaling lights
  robot_signaling_status_t get_front_red_light_state(void);

  // get the robot signaling lights
  robot_signaling_status_t get_rear_red_light_state(void);

  // get the robot signaling lights
  robot_signaling_status_t get_front_green_light_state(void);

  // get the robot signaling lights
  robot_signaling_status_t get_rear_green_light_state(void);

  // get the robot signaling lights
  robot_signaling_status_t get_left_light_state(void);

  // get the robot signaling lights
  robot_signaling_status_t get_right_light_state(void);

  // get the robot signaling lights
  robot_signaling_status_t get_buzzer_state(void);

  // get load/unload timeout
  robot_signaling_status_t get_load_unload_timeout_seconds(void);

  // get command sended to robot
  robot_command_t get_command(void);

  // get scanner config
  uint8_t get_laser_scanner_config(void);

  // get last read write operation time in microseconds
  uint64_t get_last_read_write_time(void);

  // get last read write operation time in microseconds
  uint32_t get_error_count(void);

  /* NEW - mission methods */

  // get last mission sequence number. When new number , new mission.
  uint8_t get_mission_seq(void);

  // get mission type
  uint8_t get_mission_cmd(void); // return new mission cmd and reset new command flag
  bool is_mission_cancel_cmd(void);
  bool is_mission_move_only(void);
  bool is_mission_load_at_dest(void);
  bool is_mission_unload_at_dest(void);
  bool is_mission_dock_from_rear(void);
  bool is_new_mission_cmd(void); // return true when detection of new sequence (new command flag)

  // get mission node
  void get_mission_node_label(char * label); // return the node label dest for mission

  // set mission status / feedback
  void set_mission_status(mission_status_tp status);

  // send a text message
  void send_message(char * msg /*must be a 20bytes string*/, message_tp msgtp = MESSAGE_INFO, uint8_t msgCode = 0);
  // \NEW 

};

#endif