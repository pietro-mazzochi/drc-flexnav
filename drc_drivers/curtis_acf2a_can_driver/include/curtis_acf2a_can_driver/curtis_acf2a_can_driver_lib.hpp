#ifndef CURTIS_ACF2A_CAN_DRIVER_LIB_HPP
#define CURTIS_ACF2A_CAN_DRIVER_LIB_HPP

#include "curtis_acf2a_can_driver/curtis_acf2a_can_driver_def.hpp"
#include "math.h"
#include <cmath>
#include <mutex>
#include <thread>
#define _USE_MATH_DEFINES
#define MIN2SEC 60.0

// Defines to config motor
#define CAN_READ_RATE_DEFAULT 1000
#define CAN_WRITE_RATE_DEFAULT 100

// Throttle config
#define REVERSE_MAP_SHAPE 16383 // 50%
#define THROTTLE_FILTER 1000    // 10 = 0.1Hz      1000 = 10Hz
#define HPD_SRO_TYPE 0          // 0 = Disabled
#define SEQUENCING_DELAY 0      // 0 = 0s          1250 = 5s

// Control mode select
#define CONFIG_CONTROL_MODE_DEFAULT                                            \
  4 // 0 = Speed Mode Express; 1 = Speed Mode; 2 = Torque Mode; 3 = Direct
    // Torque; 4 = Simplified Torque
// Settings for torque mode usage
#define MAX_RPM_SPEED_TORQUE_MODE                                              \
  2500 // Max speed in torque mode (RPM) (500 - 24000)
#define KP_TORQUE_MODE                                                         \
  2458 // Kp gain of torque controller (0 = 0      8192 = 100%)
#define KI_TORQUE_MODE                                                         \
  300 // Ki gain of torque controller (0 = 0      1000 = 100%)
#define KD_TORQUE_MODE                                                         \
  819 // Kd gain of torque controller (0 = 0      8192 = 100%)
#define ACCEL_RATE_TORQUE_MODE                                                 \
  100 // Accel Rate in torque mode (100 = 0.1s   30000 = 30s)
#define ACCEL_REL_RATE_TORQUE_MODE                                             \
  100 // Accel Release Rate in torque mode (100 = 0.1s   2000 = 2s)
#define GEAR_SOFTEN_TORQUE_MODE                                                \
  0 // Gear Soften torque mode (0 = 0%   5000 = 100%)
#define REVERSAL_SOFTEN_TORQUE_MODE                                            \
  0 // Reversal Soften torque mode (0 = 0%   3000 = 100%)
// Settings for speed mode express usage
#define MAX_RPM_SPEED_SPEED_MODE_EXPRESS                                       \
  2500 // Max speed in speed express mode (RPM) (500 - 24000)
#define KP_SPEED_MODE_EXPRESS                                                  \
  300 // Kp gain of speed express controller (0 = 0      1000 = 100%)
#define KI_SPEED_MODE_EXPRESS                                                  \
  300 // Ki gain of speed express controller (0 = 0      1000 = 100%)
#define ACCEL_RATE_SPEED_MODE_EXPRESS                                          \
  5000 // Accel Rate in speed express mode (100 = 0.1s   30000 = 30s)
#define DECEL_RATE_SPEED_MODE_EXPRESS                                          \
  5000 // Accel Release Rate in speed express mode (100 = 0.1s   30000 = 30s)

// Helper functions
template <typename T, typename U>
T encoder2pos(U encoder_counts, U encoder_max_counts, T gear_ratio,
              bool use_gear_ratio = false) {
  // Encoder counts to angular displacement in rad
  T pos =
      ((static_cast<T>(encoder_counts) / static_cast<T>(encoder_max_counts)) *
       static_cast<T>(2 * M_PI));
  if (use_gear_ratio)
    return pos / gear_ratio;
  return pos;
}

template <typename U> float rpm2rads(U vel) {
  // Velocity in RPM to rad/s
  return (static_cast<float>(vel) * static_cast<float>(2 * M_PI)) /
         static_cast<float>(MIN2SEC);
}

template <typename T, typename U>
float motor_rpm_to_rads(T vel, U gear_ratio, bool use_gear_ratio = false) {
  // Velocity in motor to Velocity in wheel
  if (use_gear_ratio)
    return rpm2rads(vel) / gear_ratio;
  return rpm2rads(vel);
}

//
/**
 *   @brief  Curtis_ACF2A_Can_Driver Class
 */
class Curtis_ACF2A_Can_Driver {
private:
  /**
   * @brief   CAN context variables
   */
  CANOPEN_Interface &_can;
  uint8_t controller_ID;
  uint8_t motor_swap_dir = 0;
  int control_mode_ = CONFIG_CONTROL_MODE_DEFAULT;

  /**
   * @brief   CAN TPDO variables
   */
  pdo_config_t tpdo_cfg[4] = {};
  int64_t tpdo_timestamp_ms[4] = {0, 0, 0, 0};

  /**
   * @brief   CAN RPDO config variables
   */
  pdo_config_t rpdo_cfg[4] = {};

  /**
   * @brief   CAN TSDO variables
   */
  bool sdo_received_flag = false;
  bool nmt_received_flag = false;
  sdo_can_frame sdo_received = {};

  /**
   * @brief   Motor variables
   */
  int64_t nmt_timestamp_ms = 0;
  bool motor_configured_flag = false;
  uint8_t NMT_state = COMM_TIMEOUT;
  uint8_t error_code = NOT_INITIALIZED;
  int32_t motor_encoder_count = 0;
  int16_t motor_speed_rpm = 0;
  float motor_torque_fb = 0;
  int16_t motor_thrtl_cmd = 0;

  /**
   * @brief   Encoder variables
   */
  int64_t motor_encoder_internal = 0;
  uint8_t enc_first_run = 1;
  int32_t motor_encoder_last_count = 0;

  /**
   * @brief   Mutex variables
   */
  std::mutex mtx_motor_enc;
  std::mutex mtx_motor_speed;
  std::mutex mtx_motor_tq_fb;
  std::mutex mtx_motor_tq_cmd;
  std::mutex mtx_motor_thrtl_cmd;

  /**
   * @brief   Read data thread var
   */
  std::thread read_thread_handler;

  /**
   *
   * @brief   FUnction of thread to read CAN data
   *
   */
  void read_loop_thread();

  /**
   *
   * @brief   Function to handle encoder overflow
   *
   * @param   count   Motor encoder counts
   *
   */
  void enc_ovfl_handle(int32_t count);

public:
  Curtis_ACF2A_Can_Driver(CANOPEN_Interface &_can_interface);

  /**
   *
   * @brief   Function to clean up the context of Curtis Motor controller
   *
   */
  ~Curtis_ACF2A_Can_Driver();

  /**
   *
   * @brief   Function to initialize the interface for Curtis Motor controller
   *
   * @param   _controller_ID  Motor controller ID configured in Curtis
   * @param   _motor_swap_dir Motor swap direction config (0 or 1)
   *
   */
  void init(uint8_t _controller_ID, uint8_t _motor_swap_dir);

  /**
   *
   * @brief   Function to set control mode
   *
   * @param   _control_mode Control Mode
   *
   */
  void set_config_control_mode(int _control_mode = CONFIG_CONTROL_MODE_DEFAULT);

  /**
   *
   * @brief   Function to configure Curtis Motor parameters
   *
   * @param   _log_level  Log level
   *
   */
  bool config(std::string _log_level = "error");

  /**
   *
   * @brief   Function to compute throttle reference depending on control mode
   *
   * @param   throttle input
   *
   * @return  reference throttle reference depending on control mode
   *
   */
  int16_t compute_control_reference(float throttle);

  /**
   *
   * @brief   Function to set motor torque (Nm)
   *
   * @param   _torque_setpoint  Motor torque setpoint
   *
   * @return  Success / Fail
   *
   */
  bool set_torque(float _torque_setpoint);

  /**
   *
   * @brief   Function to get motor torque feedback (Nm)
   *
   * @return  Motor torque
   *
   */
  float get_torque();

  /**
   *
   * @brief   Function to get motor speed feedback (rpm)
   *
   * @return  Motor speed
   *
   */
  int16_t get_speed();

  /**
   *
   * @brief   Function to get motor encoder counts
   *
   * @return  Motor encoder counts
   *
   */
  int64_t get_encoder();

  /**
   *
   * @brief   Function to set motor encoder counts
   *
   * @param   _enc_internal   Motor encoder counts
   *
   */
  void set_encoder(int64_t _enc_internal);

  /**
   *
   * @brief   Loop function that parses the received CAN data
   *
   * @return  Enum "curtis_parse_result"
   *
   */
  nmt_state parse_loop();

  /**
   *
   * @brief   Function to config TPDO of motor
   *
   * @param   _tpdo_number    Number of TPDO (1 - 4)
   * @param   _tpdo_cfg       Pointer to "pdo_config_t" struct
   * @param   _timeout_us     Timeout (us)
   *
   * @return  Success / Fail
   *
   */
  bool config_TPDO(uint8_t _tpdo_number, pdo_config_t *_tpdo_cfg,
                   uint32_t _timeout_us = 50000);

  /**
   *
   * @brief   Function to config RPDO of motor
   *
   * @param   _rpdo_number    Number of TPDO (1 - 4)
   * @param   _rpdo_cfg       Pointer to "pdo_config_t" struct
   * @param   _timeout_us     Timeout (us)
   *
   * @return  Success / Fail
   *
   */
  bool config_RPDO(uint8_t _rpdo_number, pdo_config_t *_rpdo_cfg,
                   uint32_t _timeout_us = 50000);

  /**
   *
   * @brief   Function to write RPDO to motor
   *
   * @param   _rpdo_number    Number of TPDO (1 - 4)
   *
   * @return  Success / Fail
   *
   */
  bool write_RPDO(uint8_t _rpdo_number);

  /**
   *
   * @brief   Function to config motor parameter using SDO
   *
   * @param   _index      Index to configure
   * @param   _sub_index  Sub-index to configure
   * @param   _value      Value to set
   * @param   _nb         Number of bytes to write
   * @param   _timeout_us Timeout (us)
   *
   * @return  Success / Fail
   *
   */
  bool config_param(uint16_t _index, uint8_t _sub_index, int32_t _value,
                    uint8_t _nb, uint32_t _timeout_us = 50000);

  /**
   *
   * @brief   Function to read motor parameter using SDO
   *
   * @param   _index      Index to configure
   * @param   _sub_index  Sub-index to configure
   * @param   _value      Value to get
   * @param   _nb         Number of bytes to read
   * @param   _timeout_us Timeout (us)
   *
   * @return  Success / Fail
   *
   */
  bool read_param(uint16_t _index, uint8_t _sub_index, int32_t *_value,
                  uint32_t _timeout_us = 50000);

  /**
   *
   * @brief   Function to wait CANOpen SDO (Transmit SDO)
   *
   * @param   _timeout_us     Timeout (us)
   *
   * @return  Success / Fail
   *
   */
  bool wait_SDO(uint32_t _timeout_us);

  /**
   *
   * @brief   Function to get NMT State of motor
   *
   * @return  NMT State
   *
   */
  nmt_state get_state();

  /**
   *
   * @brief   Function to wait NMT State of motor
   *
   * @param   _nmt_state     NMT state to wait
   * @param   _timeout_ms    Timeout (ms)
   *
   * @return  Success / Fail
   *
   */
  bool wait_state(nmt_state _nmt_state, uint16_t _timeout_ms = 100);

  /**
   *
   * @brief   Function to check state and automatic set to operational
   *
   * @param   _timeout_ms    Timeout (ms) if necessary change state
   *
   * @return  -1 fail / 0 not necessary / 1 success
   *
   */
  int8_t check_state_go_operational(uint16_t _timeout_ms = 2);

  /**
   *
   * @brief   Function to get error code of motor
   *
   * @return  Error code
   *
   */
  curtis_error_codes get_error_code();

  /**
   *
   * @brief   Function to get error message of motor
   *
   * @return  Error string
   *
   */
  const char *get_error_message();

  /**
   *
   * @brief   Function to get state message of motor
   *
   * @return  State string
   *
   */
  const char *get_state_message();

  /**
   *
   * @brief   Function to send NMT command to motor
   *
   * @param   _nmt_state  NMT command code
   *
   * @return  Success / Fail
   *
   */
  bool set_state(nmt_commands _nmt_state);
};

#endif // CURTIS_ACF2A_CAN_DRIVER_LIB_HPP
