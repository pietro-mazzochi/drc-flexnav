#ifndef CURTIS_ACF2A_CAN_DRIVER_LIB_CPP
#define CURTIS_ACF2A_CAN_DRIVER_LIB_CPP

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <mutex>

#include "curtis_acf2a_can_driver/canopen_interface.hpp"
#include "curtis_acf2a_can_driver/curtis_acf2a_can_driver_lib.hpp"

Curtis_ACF2A_Can_Driver::Curtis_ACF2A_Can_Driver(
    CANOPEN_Interface &_can_interface)
    : _can(_can_interface) {}

Curtis_ACF2A_Can_Driver::~Curtis_ACF2A_Can_Driver() {
  // Wait read CAN data thread end
  read_thread_handler.join();
}

void Curtis_ACF2A_Can_Driver::read_loop_thread(){
  while(true){
    _can.read_loop();
    parse_loop();
  }
}

void Curtis_ACF2A_Can_Driver::init(uint8_t _controller_ID, uint8_t _motor_swap_dir) {
  controller_ID = _controller_ID;
  if(_motor_swap_dir < 2)   motor_swap_dir = _motor_swap_dir;
  tpdo_cfg[0].count = 0;
  tpdo_cfg[1].count = 0;
  tpdo_cfg[2].count = 0;
  tpdo_cfg[3].count = 0;
  rpdo_cfg[0].count = 0;
  rpdo_cfg[1].count = 0;
  rpdo_cfg[2].count = 0;
  rpdo_cfg[3].count = 0;

  // Read CAN data thread init
  read_thread_handler = std::thread(&Curtis_ACF2A_Can_Driver::read_loop_thread, this);
};

void Curtis_ACF2A_Can_Driver::set_config_control_mode(int _control_mode) {
  control_mode_ = _control_mode;
}

bool Curtis_ACF2A_Can_Driver::config(std::string _log_level) {
  bool configuration_succeded = true;
  // Log level config
  std::cerr.clear();
  std::cout.clear();
  int lg_lv = 0;
  if (_log_level == "error")
    std::cout.setstate(std::ios_base::failbit);
  else if (_log_level == "verbose")
    lg_lv = 1;

  // Reset motor and wait for boot up
  set_state(GO_RESET_NODE);
  if (wait_state(BOOT_UP, BOOT_UP_WAIT_TIME_MS)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID << " motor boot up!"
                << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID << " motor FAIL first boot up!"
                << std::endl;
  }
  sleep(1);
  // Enable save parameters on driver
  if (config_param(CURTIS_NMV_WRITE_EN_INDEX, CURTIS_NMV_WRITE_EN_SUB_INDEX, 1,
                   4)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor NVM Write Enable success!" << std::endl;
  } else {
    //WARN: it's not working on mottor 0x25
    //configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor NVM Write Enable fail!" << std::endl;
  }
  // Config motors TPDO1
  pdo_config_t tpdo1_cfg = {.count = TPDO1_PARAMETERS_COUNT};
  tpdo1_cfg.sizes[0] = sizeof(motor_encoder_count);
  tpdo1_cfg.data_ptr[0] = &motor_encoder_count;
  tpdo1_cfg.index[0] = CURTIS_ENCODER_COUNT_INDEX;
  tpdo1_cfg.sub_index[0] = CURTIS_ENCODER_COUNT_SUB_INDEX;
  tpdo1_cfg.sizes[1] = sizeof(motor_speed_rpm);
  tpdo1_cfg.data_ptr[1] = &motor_speed_rpm;
  tpdo1_cfg.index[1] = CURTIS_RPM_INDEX;
  tpdo1_cfg.sub_index[1] = CURTIS_RPM_SUB_INDEX;
  tpdo1_cfg.timeout_ms =
      TPDO1_READ_TIME_MS; // Minimum time interval on Curtis is 4ms (0 = 4ms)
  if (config_TPDO(TPDO1, &tpdo1_cfg)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor TPDO1 config success!" << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor TPDO1 config fail!" << std::endl;
  }

  // Config motors TPDO2
  pdo_config_t tpdo2_cfg = {.count = TPDO2_PARAMETERS_COUNT};
  tpdo2_cfg.sizes[0] = sizeof(motor_torque_fb);
  tpdo2_cfg.data_ptr[0] = &motor_torque_fb;
  tpdo2_cfg.index[0] = CURTIS_TORQUE_INDEX;
  tpdo2_cfg.sub_index[0] = CURTIS_TORQUE_SUB_INDEX;
  tpdo2_cfg.timeout_ms =
      TPDO2_READ_TIME_MS; // Minimum time interval on Curtis is 4ms (0 = 4ms)
  if (config_TPDO(TPDO2, &tpdo2_cfg)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor TPDO2 config success!" << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor TPDO2 config fail!" << std::endl;
  }

  // Disable motors TPDO3 e TPDO4
  pdo_config_t tpdo_disable = {.count = PDO_DISABLE};
  if (config_TPDO(TPDO3, &tpdo_disable)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor TPDO3 disable success!" << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor TPDO3 disable fail!" << std::endl;
  }
  if (config_TPDO(TPDO4, &tpdo_disable)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor TPDO4 disable success!" << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor TPDO4 disable fail!" << std::endl;
  }
  // Config motor RPDO1
  pdo_config_t rpdo1_cfg = {.count = RPDO1_PARAMETERS_COUNT};
  rpdo1_cfg.sizes[0] = sizeof(motor_thrtl_cmd);
  rpdo1_cfg.data_ptr[0] = &motor_thrtl_cmd;
  rpdo1_cfg.index[0] = CURTIS_VCL_THROTTLE_INDEX;
  rpdo1_cfg.sub_index[0] = CURTIS_VCL_THROTTLE_SUB_INDEX;
  rpdo1_cfg.timeout_ms = RPDO1_TIMEOUT_MS;
  if (config_RPDO(RPDO1, &rpdo1_cfg)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor RPDO1 config success!" << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor RPDO1 config fail!" << std::endl;
  }
  // Disable motor RPDO2, RPDO3 e RPDO4
  pdo_config_t rpdo_disable = {.count = PDO_DISABLE};
  if (config_RPDO(RPDO2, &rpdo_disable)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor RPDO2 disable success!" << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor RPDO2 disable fail!" << std::endl;
  }
  if (config_RPDO(RPDO3, &rpdo_disable)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor RPDO3 disable success!" << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor RPDO3 disable fail!" << std::endl;
  }
  if (config_RPDO(RPDO4, &rpdo_disable)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor RPDO4 disable success!" << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor RPDO4 disable fail!" << std::endl;
  }
  if (config_param(CURTIS_REV_MAP_SHAPE_INDEX, CURTIS_REV_MAP_SHAPE_SUB_INDEX,
                   REVERSE_MAP_SHAPE, 2)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor Reverse Map Shape config success!" << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor Reverse Map Shape config fail!" << std::endl;
  }
  if (config_param(CURTIS_THROTTLE_FILTER_INDEX,
                   CURTIS_THROTTLE_FILTER_SUB_INDEX, THROTTLE_FILTER, 2)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor Throttle Filter config success!" << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor Throttle Filter config fail!" << std::endl;
  }
  if (config_param(CURTIS_HPD_SRO_TYPE_INDEX, CURTIS_HPD_SRO_TYPE_SUB_INDEX,
                   HPD_SRO_TYPE, 2)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor HPD SRO Type config success!" << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor HPD SRO Type config fail!" << std::endl;
  }
  if (config_param(CURTIS_SEQUENC_DELAY_INDEX, CURTIS_SEQUENC_DELAY_SUB_INDEX,
                   SEQUENCING_DELAY, 2)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor Sequencing Delay config success!" << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor Sequencing Delay config fail!" << std::endl;
  }

  // Config motor parameters
  if (config_param(CURTIS_CONTROL_MODE_INDEX, CURTIS_CONTROL_MODE_SUB_INDEX,
                   control_mode_, 2)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor Control Mode config success!" << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor Control Mode config fail!" << std::endl;
  }
  switch (control_mode_) {
  case 0: // Speed Mode Express
    std::cout << "0x" << std::hex << (int)controller_ID
              << " Control mode set to Speed Express!" << std::endl;
    if (config_param(CURTIS_MAX_SPEED_SPEED_MD_EX_INDEX,
                     CURTIS_MAX_SPEED_SPEED_MD_EX_SUB_INDEX,
                     MAX_RPM_SPEED_SPEED_MODE_EXPRESS, 2)) {
      if (lg_lv > 0)
        std::cout << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Max_speed config success!" << std::endl;
    } else {
      configuration_succeded = false;
      if (lg_lv > 0)
        std::cerr << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Max_speed config fail!" << std::endl;
    }
    if (config_param(CURTIS_KP_SPEED_MD_EX_INDEX,
                     CURTIS_KP_SPEED_MD_EX_SUB_INDEX, KP_SPEED_MODE_EXPRESS,
                     2)) {
      if (lg_lv > 0)
        std::cout << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Kp config success!" << std::endl;
    } else {
      configuration_succeded = false;
      if (lg_lv > 0)
        std::cerr << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Kp config fail!" << std::endl;
    }
    if (config_param(CURTIS_KI_SPEED_MD_EX_INDEX,
                     CURTIS_KI_SPEED_MD_EX_SUB_INDEX, KI_SPEED_MODE_EXPRESS,
                     2)) {
      if (lg_lv > 0)
        std::cout << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Ki config success!" << std::endl;
    } else {
      configuration_succeded = false;
      if (lg_lv > 0)
        std::cerr << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Ki config fail!" << std::endl;
    }
    if (config_param(CURTIS_ACCEL_RATE_SPEED_MD_EX_INDEX,
                     CURTIS_ACCEL_RATE_SPEED_MD_EX_SUB_INDEX,
                     ACCEL_RATE_SPEED_MODE_EXPRESS, 2)) {
      if (lg_lv > 0)
        std::cout << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Accel Rate config success!" << std::endl;
    } else {
      configuration_succeded = false;
      if (lg_lv > 0)
        std::cerr << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Accel Rate config fail!" << std::endl;
    }
    if (config_param(CURTIS_DECEL_RATE_SPEED_MD_EX_INDEX,
                     CURTIS_DECEL_RATE_SPEED_MD_EX_SUB_INDEX,
                     DECEL_RATE_SPEED_MODE_EXPRESS, 2)) {
      if (lg_lv > 0)
        std::cout << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Decel Rate config success!" << std::endl;
    } else {
      configuration_succeded = false;
      if (lg_lv > 0)
        std::cerr << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Decel Rate config fail!" << std::endl;
    }
    break;

  case 1:
    std::cout << "0x" << std::hex << (int)controller_ID
              << " Control mode set to Speed!" << std::endl;

    break;

  case 2: // Torque Mode
  case 4: // Simplified Torque Mode
    std::cout << "0x" << std::hex << (int)controller_ID
              << " Control mode set to Torque/Simplified Torque!" << std::endl;
    if (config_param(CURTIS_MAX_SPEED_TORQUE_MD_INDEX,
                     CURTIS_MAX_SPEED_TORQUE_MD_SUB_INDEX,
                     MAX_RPM_SPEED_TORQUE_MODE, 2)) {
      if (lg_lv > 0)
        std::cout << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Max_speed config success!" << std::endl;
    } else {
      configuration_succeded = false;
      if (lg_lv > 0)
        std::cerr << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Max_speed config fail!" << std::endl;
    }
    if (config_param(CURTIS_KP_TORQUE_MD_INDEX, CURTIS_KP_TORQUE_MD_SUB_INDEX,
                     KP_TORQUE_MODE, 2)) {
      if (lg_lv > 0)
        std::cout << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Kp config success!" << std::endl;
    } else {
      configuration_succeded = false;
      if (lg_lv > 0)
        std::cerr << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Kp config fail!" << std::endl;
    }
    if (config_param(CURTIS_KI_TORQUE_MD_INDEX, CURTIS_KI_TORQUE_MD_SUB_INDEX,
                     KI_TORQUE_MODE, 2)) {
      if (lg_lv > 0)
        std::cout << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Ki config success!" << std::endl;
    } else {
      configuration_succeded = false;
      if (lg_lv > 0)
        std::cerr << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Ki config fail!" << std::endl;
    }
    if (config_param(CURTIS_KD_TORQUE_MD_INDEX, CURTIS_KD_TORQUE_MD_SUB_INDEX,
                     KD_TORQUE_MODE, 2)) {
      if (lg_lv > 0)
        std::cout << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Kd config success!" << std::endl;
    } else {
      configuration_succeded = false;
      if (lg_lv > 0)
        std::cerr << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Kd config fail!" << std::endl;
    }
    if (config_param(CURTIS_ACCEL_RATE_TQ_MD_INDEX,
                     CURTIS_ACCEL_RATE_TQ_MD_SUB_INDEX, ACCEL_RATE_TORQUE_MODE,
                     2)) {
      if (lg_lv > 0)
        std::cout << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Accel Rate config success!" << std::endl;
    } else {
      configuration_succeded = false;
      if (lg_lv > 0)
        std::cerr << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Accel Rate config fail!" << std::endl;
    }
    if (config_param(CURTIS_ACCEL_REL_RATE_TQ_MD_INDEX,
                     CURTIS_ACCEL_REL_RATE_TQ_MD_SUB_INDEX,
                     ACCEL_REL_RATE_TORQUE_MODE, 2)) {
      if (lg_lv > 0)
        std::cout << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Accel Release Rate config success!"
                  << std::endl;
    } else {
      configuration_succeded = false;
      if (lg_lv > 0)
        std::cerr << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Accel Release Rate config fail!"
                  << std::endl;
    }
    if (config_param(CURTIS_GEAR_SOFTEN_TQ_MD_INDEX,
                     CURTIS_GEAR_SOFTEN_TQ_MD_SUB_INDEX,
                     GEAR_SOFTEN_TORQUE_MODE, 2)) {
      if (lg_lv > 0)
        std::cout << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Gear Soften config success!"
                  << std::endl;
    } else {
      configuration_succeded = false;
      if (lg_lv > 0)
        std::cerr << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Gear Soften config fail!" << std::endl;
    }
    if (config_param(CURTIS_REVERSAL_SOFTEN_TQ_MD_INDEX,
                     CURTIS_REVERSAL_SOFTEN_TQ_MD_SUB_INDEX,
                     REVERSAL_SOFTEN_TORQUE_MODE, 2)) {
      if (lg_lv > 0)
        std::cout << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Reversal Soften config success!"
                  << std::endl;
    } else {
      configuration_succeded = false;
      if (lg_lv > 0)
        std::cerr << "0x" << std::hex << (int)controller_ID
                  << " motor Parameter Reversal Soften config fail!"
                  << std::endl;
    }
    break;

  default:

    break;
  }

  // Config motor direction
  if (config_param(CURTIS_SWAP_MOTOR_DIR_INDEX, CURTIS_SWAP_MOTOR_DIR_SUB_INDEX, motor_swap_dir, 2)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor swap direction write success!" << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor swap direction write fail!" << std::endl;
  }

  // Disable save parameters on driver
  if (config_param(CURTIS_NMV_WRITE_EN_INDEX, CURTIS_NMV_WRITE_EN_SUB_INDEX, 0,
                   4)) {
    if (lg_lv > 0)
      std::cout << "0x" << std::hex << (int)controller_ID
                << " motor NVM Write Disable success!" << std::endl;
  } else {
    configuration_succeded = false;
    if (lg_lv > 0)
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " motor NVM Write Disable fail!" << std::endl;
  }

  // Reset motor and wait for boot up
  set_state(GO_RESET_NODE);
  if (wait_state(BOOT_UP, BOOT_UP_WAIT_TIME_MS)) {
    std::cout << "0x" << std::hex << (int)controller_ID << " motor boot up!"
              << std::endl;
  } else {
    configuration_succeded = false;
    std::cerr << "0x" << std::hex << (int)controller_ID << " motor FAIL final boot up!"
              << std::endl;
  }
  // Set motors state to operational
  if (get_state() != OPERATIONAL) {
    set_state(GO_OPERATIONAL);
  }

  if (wait_state(OPERATIONAL)) {
    std::cerr << "0x" << std::hex << (int)controller_ID << " motor operational!"
              << std::endl;
  } else {
    configuration_succeded = false;
    std::cerr << "Fail to set 0x" << std::hex << (int)controller_ID
              << " motor to operational!" << std::endl;
  }

  // motor_configured_flag = configuration_succeded;
  motor_configured_flag = true;

  return motor_configured_flag;
}

int16_t Curtis_ACF2A_Can_Driver::compute_control_reference(float throttle) {
  int16_t reference;

  switch (control_mode_) {
  case 0:
  case 1:
    // speed express, speed mode
    reference =
        throttle / 2500.0 *
        32767; // RPM speed mode, 16 bits (-32768 a 32767), (-2500, 2500) RPM
    break;
  case 2:
    // torque mode
    reference =
        throttle / 5.6 * 32767; // Torque mode, (-32768 a 32767), (-5.6, 5.6) Nm
    break;
  case 4:
    // simplified torque mode
    if (throttle > 0.6) {
      reference = (int16_t)((throttle * 2224.685) +
                            2778.6); // Simplified Torque mode, (-32768 a 32767)
    } else if (throttle > 0) {
      reference =
          (int16_t)((38440.263 * pow(throttle, 3)) -
                    (40840.608 * pow(throttle, 2)) + (17793.368 * throttle) +
                    148.92); // Simplified Torque mode, (-32768 a 32767)
    } else if (throttle < -0.6) {
      reference = (int16_t)((throttle * 2224.685) -
                            2778.6); // Simplified Torque mode, (-32768 a 32767)
    } else if ((throttle < 0)) {
      reference =
          (int16_t)((38440.263 * pow(throttle, 3)) +
                    (40840.608 * pow(throttle, 2)) + (17793.368 * throttle) -
                    148.92); // Simplified Torque mode, (-32768 a 32767)
    } else {
      reference = 0; // Simplified Torque mode, (-32768 a 32767)
    }
    break;
  default:
    reference = 0; // Control not defined
    break;
  }
  return reference;
}

bool Curtis_ACF2A_Can_Driver::set_torque(float _torque_setpoint) { 
  // Get motor throttle command mutex
  std::lock_guard<std::mutex> lock_thrtl_cmd(mtx_motor_thrtl_cmd);
  // Calculate throttle command
  motor_thrtl_cmd = compute_control_reference(_torque_setpoint);
  // Write command to CAN port
  return write_RPDO(1); 
}

float Curtis_ACF2A_Can_Driver::get_torque() {
  // Get motor torque feedback mutex
  std::lock_guard<std::mutex> lock_tq_fb(mtx_motor_tq_fb);
  return motor_torque_fb;
}

int16_t Curtis_ACF2A_Can_Driver::get_speed() {
  // Get motor speed feedback mutex
  std::lock_guard<std::mutex> lock_speed(mtx_motor_speed);
  return motor_speed_rpm;
}

int64_t Curtis_ACF2A_Can_Driver::get_encoder() {
  // Get motor encoder mutex
  std::lock_guard<std::mutex> lock_enc(mtx_motor_enc);
  enc_ovfl_handle(motor_encoder_count);
  return motor_encoder_internal;
}

void Curtis_ACF2A_Can_Driver::set_encoder(int64_t _enc_internal ) {
  // Get motor encoder mutex
  std::lock_guard<std::mutex> lock_enc(mtx_motor_enc);
  motor_encoder_internal = _enc_internal;
}

void Curtis_ACF2A_Can_Driver::enc_ovfl_handle( int32_t count ){
    if( NMT_state == OPERATIONAL || NMT_state == PRE_OPERATIONAL ){
      if( enc_first_run ){
        enc_first_run = 0;
        //motor_encoder_internal = 0;
      }
      else{
          int64_t delta_count = count - motor_encoder_last_count;

          if (delta_count > (INT32_MAX / 2)) {
              delta_count -= ((int64_t)UINT32_MAX + 1);
          }
          else if (delta_count < -(INT32_MAX / 2)) {
              delta_count += ((int64_t)UINT32_MAX + 1);
          }

          motor_encoder_internal += delta_count;
      } 

      motor_encoder_last_count = count;
    }
    
    return;
}

nmt_state Curtis_ACF2A_Can_Driver::parse_loop() {
  can_frame _frame;
  uint8_t fc = 0x00;

  int8_t tpdo_index = -1;
  uint8_t index = 0;
  uint32_t var = 0;

  int64_t time_now =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::high_resolution_clock::now().time_since_epoch())
          .count();

  // Get motor torque feedback mutex
  std::lock_guard<std::mutex> lock_tq_fb(mtx_motor_tq_fb);
  // Get motor speed feedback mutex
  std::lock_guard<std::mutex> lock_speed(mtx_motor_speed);
  // Get motor encoder feedback mutex
  std::lock_guard<std::mutex> lock_enc(mtx_motor_enc);

  while (_can.read_fifo(controller_ID, &_frame)) {
    tpdo_index = -1;

    fc = (_frame.can_id >> 7) & 0x0F;

    switch (fc) {
    case FC_HEARTBEAT:
      nmt_received_flag = true;
      nmt_timestamp_ms = time_now;
      NMT_state = _frame.data[0];
      break;

    case FC_TPDO1:
      tpdo_timestamp_ms[0] = time_now;
      tpdo_index = TPDO_INDEX_1;
      break;

    case FC_TPDO2:
      tpdo_timestamp_ms[1] = time_now;
      tpdo_index = TPDO_INDEX_2;
      break;

    case FC_TPDO3:
      tpdo_timestamp_ms[2] = time_now;
      tpdo_index = TPDO_INDEX_3;
      break;

    case FC_TPDO4:
      tpdo_timestamp_ms[3] = time_now;
      tpdo_index = TPDO_INDEX_4;
      break;

    case FC_TSDO:
      sdo_received_flag = true;

      sdo_received.cob_ID = _frame.can_id;
      sdo_received.s = _frame.data[0] & 0x01;
      sdo_received.e = (_frame.data[0] >> 1) & 0x01;
      sdo_received.n = (_frame.data[0] >> 2) & 0x03;
      sdo_received.ccs = (_frame.data[0] >> 5) & 0x07;
      sdo_received.num_data_bytes = _frame.can_dlc - 4;

      sdo_received.index = (_frame.data[2] << 8) | _frame.data[1];
      sdo_received.sub_index = _frame.data[3];

      for (uint8_t _i = 0; _i < sdo_received.num_data_bytes; _i++) {
        sdo_received.data[_i] = _frame.data[4 + _i];
      }

      break;

    case FC_EMCY:
      error_code = _frame.data[0];

      break;

    default:

      break;
    }

    if (tpdo_index >= 0) {
      index = 0;
      for (uint8_t _i = 0; _i < tpdo_cfg[tpdo_index].count; _i++) {
        var = 0;
        for (uint8_t _j = 0; _j < tpdo_cfg[tpdo_index].sizes[_i]; _j++) {
          var += _frame.data[index] << (_j << 3);
          index++;
        }
        memcpy(tpdo_cfg[tpdo_index].data_ptr[_i], &var,
               tpdo_cfg[tpdo_index].sizes[_i]);
      }
    }
  }

  // Heartbeat TIMEOUT CHECK
  if ((time_now - nmt_timestamp_ms) > 200) {
    NMT_state = COMM_TIMEOUT;
  } else {
    check_state_go_operational();
  }

  if (motor_configured_flag) {
    // Waiting TPDO1 (TIMEOUT CHECK)
    if (tpdo_cfg[0].count > 0) {
      if (tpdo_timestamp_ms[0] > 0) {
        if ((time_now - tpdo_timestamp_ms[0]) > (tpdo_cfg[0].timeout_ms + 20)) {
          // Timeout TPDO1
          NMT_state = COMM_TIMEOUT;
        }
      }
    }
    // Waiting TPDO2 (TIMEOUT CHECK)
    if (tpdo_cfg[1].count > 0) {
      if (tpdo_timestamp_ms[1] > 0) {
        if ((time_now - tpdo_timestamp_ms[1]) > (tpdo_cfg[1].timeout_ms + 20)) {
          // Timeout TPDO2
          NMT_state = COMM_TIMEOUT;
        }
      }
    }
    // Waiting TPDO3 (TIMEOUT CHECK)
    if (tpdo_cfg[2].count > 0) {
      if (tpdo_timestamp_ms[2] > 0) {
        if ((time_now - tpdo_timestamp_ms[2]) > (tpdo_cfg[2].timeout_ms + 20)) {
          // Timeout TPDO3
          NMT_state = COMM_TIMEOUT;
        }
      }
    }
    // Waiting TPDO4 (TIMEOUT CHECK)
    if (tpdo_cfg[3].count > 0) {
      if (tpdo_timestamp_ms[3] > 0) {
        if ((time_now - tpdo_timestamp_ms[3]) > (tpdo_cfg[3].timeout_ms + 20)) {
          // Timeout TPDO4
          NMT_state = COMM_TIMEOUT;
        }
      }
    }
  }

  if(NMT_state == COMM_TIMEOUT){
    motor_speed_rpm = 0;
    motor_torque_fb = 0;
    enc_first_run = 1;
    //motor_encoder_count = 0;
    //motor_encoder_internal = 0;
  }

  return (nmt_state)NMT_state;
}

bool Curtis_ACF2A_Can_Driver::config_TPDO(uint8_t _tpdo_number,
                                          pdo_config_t *_tpdo_cfg,
                                          uint32_t _timeout_us) {
  if (_tpdo_number < 1 || _tpdo_number > 4)
    return false;
  if (_tpdo_cfg->count > 8)
    return false;

  uint8_t total_bytes = 0;
  for (uint8_t _i = 0; _i < _tpdo_cfg->count; _i++) {
    if (_tpdo_cfg->sizes[_i] < 1 || _tpdo_cfg->sizes[_i] > 4)
      return false;
    total_bytes += _tpdo_cfg->sizes[_i];
  }
  if (total_bytes > 8)
    return false;

  // Copy to config map var
  memcpy(&tpdo_cfg[_tpdo_number - 1], _tpdo_cfg, sizeof(struct pdo_config_t));

  // TPDO COB IDs
  sdo_can_frame tpdo_cob_ids = {
      .cob_ID = (uint16_t)(TPDO_COB_ID + controller_ID),
      .ccs = 1,
      .n = 0,
      .e = 1,
      .s = 1,
      .index = (uint16_t)(TPDO_COB_ID_FIRST_INDEX - 1 + _tpdo_number),
      .sub_index = TPDO_COB_ID_SUB_INDEX,
      .num_data_bytes = 4};
  uint16_t tpdo_cobid = (_tpdo_number * 0x100) + 0x80 + controller_ID;
  tpdo_cob_ids.data[0] = tpdo_cobid;
  tpdo_cob_ids.data[1] = tpdo_cobid >> 8;
  tpdo_cob_ids.data[2] = 0x00;
  tpdo_cob_ids.data[3] = TPDO_COB_ID_DISABLE_BYTE;

  // TPDO lenght
  sdo_can_frame tpdo_lenght = {
      .cob_ID = (uint16_t)(TPDO_COB_ID + controller_ID),
      .ccs = 1,
      .n = 3,
      .e = 1,
      .s = 1,
      .index = (uint16_t)(TPDO_LENGTH_MAP_FIRST_INDEX - 1 + _tpdo_number),
      .sub_index = TPDO_LENGTH_SUB_INDEX,
      .num_data_bytes = 1};
  tpdo_lenght.data[0] = 0x00;

  // TPDO object Map
  sdo_can_frame tpdo_obj_map = {
      .cob_ID = (uint16_t)(TPDO_COB_ID + controller_ID),
      .ccs = 1,
      .n = 0,
      .e = 1,
      .s = 1,
      .index = (uint16_t)(TPDO_LENGTH_MAP_FIRST_INDEX - 1 + _tpdo_number),
      .num_data_bytes = 4};

  // TPDO timeout
  sdo_can_frame tpdo_time = {
      .cob_ID = (uint16_t)(TPDO_COB_ID + controller_ID),
      .ccs = 1,
      .n = 2,
      .e = 1,
      .s = 1,
      .index = (uint16_t)(TPDO_COB_ID_FIRST_INDEX - 1 + _tpdo_number),
      .sub_index = TPDO_TIMEOUT_SUB_INDEX,
      .num_data_bytes = 2};
  if (_tpdo_cfg->timeout_ms < 4)
    _tpdo_cfg->timeout_ms = 4;
  if ((_tpdo_cfg->timeout_ms % 4) != 0)
    _tpdo_cfg->timeout_ms = _tpdo_cfg->timeout_ms - (_tpdo_cfg->timeout_ms % 4);
  tpdo_time.data[0] = _tpdo_cfg->timeout_ms;
  tpdo_time.data[1] = _tpdo_cfg->timeout_ms >> 8;

  // Disable TPDO
  if (!_can.write_SDO(&tpdo_cob_ids))
    return false;
  if (!wait_SDO(_timeout_us))
    return false;
  if (sdo_received.index != tpdo_cob_ids.index)
    return false;
  if (sdo_received.sub_index != tpdo_cob_ids.sub_index)
    return false;
  if (sdo_received.ccs != 0x03)
    return false;

  // Send config to motor
  if (_tpdo_cfg->count > 0) {
    // Config TPDO lenght to 0
    if (!_can.write_SDO(&tpdo_lenght))
      return false;
    if (!wait_SDO(_timeout_us))
      return false;
    if (sdo_received.index != tpdo_lenght.index)
      return false;
    if (sdo_received.sub_index != tpdo_lenght.sub_index)
      return false;
    if (sdo_received.ccs != 0x03)
      return false;

    // Config TPDO Map of objects
    for (uint8_t _i = 0; _i < _tpdo_cfg->count; _i++) {
      tpdo_obj_map.sub_index = _i + 1;
      tpdo_obj_map.data[0] = (uint8_t)(_tpdo_cfg->sizes[_i] << 3);
      tpdo_obj_map.data[1] = _tpdo_cfg->sub_index[_i];
      tpdo_obj_map.data[2] = _tpdo_cfg->index[_i];
      tpdo_obj_map.data[3] = _tpdo_cfg->index[_i] >> 8;

      // Send Object Map
      if (!_can.write_SDO(&tpdo_obj_map))
        return false;
      if (!wait_SDO(_timeout_us))
        return false;
      if (sdo_received.index != tpdo_obj_map.index)
        return false;
      if (sdo_received.sub_index != tpdo_obj_map.sub_index)
        return false;
      if (sdo_received.ccs != 0x03)
        return false;
    }

    // Config TPDO lenght
    tpdo_lenght.data[0] = _tpdo_cfg->count;
    if (!_can.write_SDO(&tpdo_lenght))
      return false;
    if (!wait_SDO(_timeout_us))
      return false;
    if (sdo_received.index != tpdo_lenght.index)
      return false;
    if (sdo_received.sub_index != tpdo_lenght.sub_index)
      return false;
    if (sdo_received.ccs != 0x03)
      return false;

    // Config TPDO time
    if (!_can.write_SDO(&tpdo_time))
      return false;
    if (!wait_SDO(_timeout_us))
      return false;
    if (sdo_received.index != tpdo_time.index)
      return false;
    if (sdo_received.sub_index != tpdo_time.sub_index)
      return false;
    if (sdo_received.ccs != 0x03)
      return false;

    // Enable TPDO
    tpdo_cob_ids.data[3] = TPDO_COB_ID_ENABLE_BYTE;
    if (!_can.write_SDO(&tpdo_cob_ids))
      return false;
    if (!wait_SDO(_timeout_us))
      return false;
    if (sdo_received.index != tpdo_cob_ids.index)
      return false;
    if (sdo_received.sub_index != tpdo_cob_ids.sub_index)
      return false;
    if (sdo_received.ccs != 0x03)
      return false;
  }

  return true;
}

bool Curtis_ACF2A_Can_Driver::config_RPDO(uint8_t _rpdo_number,
                                          pdo_config_t *_rpdo_cfg,
                                          uint32_t _timeout_us) {
  if (_rpdo_number < 1 || _rpdo_number > 4)
    return false;
  if (_rpdo_cfg->count > 8)
    return false;

  uint8_t total_bytes = 0;
  for (uint8_t _i = 0; _i < _rpdo_cfg->count; _i++) {
    if (_rpdo_cfg->sizes[_i] < 1 || _rpdo_cfg->sizes[_i] > 4)
      return false;
    total_bytes += _rpdo_cfg->sizes[_i];
  }
  if (total_bytes > 8)
    return false;

  // Copy to config map var
  memcpy(&rpdo_cfg[_rpdo_number - 1], _rpdo_cfg, sizeof(struct pdo_config_t));

  // RPDO COB IDs
  sdo_can_frame rpdo_cob_ids = {
      .cob_ID = (uint16_t)(TPDO_COB_ID + controller_ID),
      .ccs = 1,
      .n = 0,
      .e = 1,
      .s = 1,
      .index = (uint16_t)(RPDO_COB_ID_FIRST_INDEX - 1 + _rpdo_number),
      .sub_index = RPDO_COB_ID_SUB_INDEX,
      .num_data_bytes = 4};
  uint16_t rpdo_cobid = (_rpdo_number * 0x100) + 0x100 + controller_ID;
  rpdo_cob_ids.data[0] = rpdo_cobid;
  rpdo_cob_ids.data[1] = rpdo_cobid >> 8;
  rpdo_cob_ids.data[2] = 0x00;
  rpdo_cob_ids.data[3] = RPDO_COB_ID_DISABLE_BYTE;

  // RPDO lenght
  sdo_can_frame rpdo_lenght = {
      .cob_ID = (uint16_t)(TPDO_COB_ID + controller_ID),
      .ccs = 1,
      .n = 3,
      .e = 1,
      .s = 1,
      .index = (uint16_t)(RPDO_LENGTH_MAP_FIRST_INDEX - 1 + _rpdo_number),
      .sub_index = RPDO_LENGTH_SUB_INDEX,
      .num_data_bytes = 1};
  rpdo_lenght.data[0] = 0x00;

  // RPDO object Map
  sdo_can_frame rpdo_obj_map = {
      .cob_ID = (uint16_t)(TPDO_COB_ID + controller_ID),
      .ccs = 1,
      .n = 0,
      .e = 1,
      .s = 1,
      .index = (uint16_t)(RPDO_LENGTH_MAP_FIRST_INDEX - 1 + _rpdo_number),
      .num_data_bytes = 4};

  // RPDO timeout
  sdo_can_frame rpdo_time = {
      .cob_ID = (uint16_t)(TPDO_COB_ID + controller_ID),
      .ccs = 1,
      .n = 2,
      .e = 1,
      .s = 1,
      .index = (uint16_t)(RPDO_COB_ID_FIRST_INDEX - 1 + _rpdo_number),
      .sub_index = RPDO_TIMEOUT_SUB_INDEX,
      .num_data_bytes = 2};
  rpdo_time.data[0] = _rpdo_cfg->timeout_ms;
  rpdo_time.data[1] = _rpdo_cfg->timeout_ms >> 8;

  // Disable RPDO
  if (!_can.write_SDO(&rpdo_cob_ids))
    return false;
  if (!wait_SDO(_timeout_us))
    return false;
  if (sdo_received.index != rpdo_cob_ids.index)
    return false;
  if (sdo_received.sub_index != rpdo_cob_ids.sub_index)
    return false;
  if (sdo_received.ccs != 0x03)
    return false;

  // Send config to motor
  if (_rpdo_cfg->count > 0) {
    // Config RPDO lenght to 0
    if (!_can.write_SDO(&rpdo_lenght))
      return false;
    if (!wait_SDO(_timeout_us))
      return false;
    if (sdo_received.index != rpdo_lenght.index)
      return false;
    if (sdo_received.sub_index != rpdo_lenght.sub_index)
      return false;
    if (sdo_received.ccs != 0x03)
      return false;

    // Config RPDO Map of objects
    for (uint8_t _i = 0; _i < _rpdo_cfg->count; _i++) {
      rpdo_obj_map.sub_index = _i + 1;
      rpdo_obj_map.data[0] = (uint8_t)(_rpdo_cfg->sizes[_i] << 3);
      rpdo_obj_map.data[1] = _rpdo_cfg->sub_index[_i];
      rpdo_obj_map.data[2] = _rpdo_cfg->index[_i];
      rpdo_obj_map.data[3] = _rpdo_cfg->index[_i] >> 8;

      // Send Object Map
      if (!_can.write_SDO(&rpdo_obj_map))
        return false;
      if (!wait_SDO(_timeout_us))
        return false;
      if (sdo_received.index != rpdo_obj_map.index)
        return false;
      if (sdo_received.sub_index != rpdo_obj_map.sub_index)
        return false;
      if (sdo_received.ccs != 0x03)
        return false;
    }

    // Config RPDO lenght
    rpdo_lenght.data[0] = _rpdo_cfg->count;
    if (!_can.write_SDO(&rpdo_lenght))
      return false;
    if (!wait_SDO(_timeout_us))
      return false;
    if (sdo_received.index != rpdo_lenght.index)
      return false;
    if (sdo_received.sub_index != rpdo_lenght.sub_index)
      return false;
    if (sdo_received.ccs != 0x03)
      return false;

    // Config RPDO time
    if (!_can.write_SDO(&rpdo_time))
      return false;
    if (!wait_SDO(_timeout_us))
      return false;
    if (sdo_received.index != rpdo_time.index)
      return false;
    if (sdo_received.sub_index != rpdo_time.sub_index)
      return false;
    if (sdo_received.ccs != 0x03)
      return false;

    // Enable RPDO
    rpdo_cob_ids.data[3] = RPDO_COB_ID_ENABLE_BYTE;
    if (!_can.write_SDO(&rpdo_cob_ids))
      return false;
    if (!wait_SDO(_timeout_us))
      return false;
    if (sdo_received.index != rpdo_cob_ids.index)
      return false;
    if (sdo_received.sub_index != rpdo_cob_ids.sub_index)
      return false;
    if (sdo_received.ccs != 0x03)
      return false;
  }

  return true;
}

bool Curtis_ACF2A_Can_Driver::write_RPDO(uint8_t _rpdo_number) {
  if (_rpdo_number < 1 || _rpdo_number > 4)
    return false;
  uint8_t _index = 0;

  pdo_can_frame _rpdo_write = {
      .cob_ID = (uint16_t)((_rpdo_number * 0x100) + 0x100 + controller_ID),
      .data_size = 0};

  for (uint8_t _i = 0; _i < rpdo_cfg[_rpdo_number - 1].count; _i++) {
    memcpy(&_rpdo_write.data[_index], rpdo_cfg[_rpdo_number - 1].data_ptr[_i],
           rpdo_cfg[_rpdo_number - 1].sizes[_i]);
    _index += rpdo_cfg[_rpdo_number - 1].sizes[_i];

    _rpdo_write.data_size += rpdo_cfg[_rpdo_number - 1].sizes[_i];
  }

  if (!_can.write_RPDO(&_rpdo_write))
    return false;

  return true;
}

bool Curtis_ACF2A_Can_Driver::config_param(uint16_t _index, uint8_t _sub_index,
                                           int32_t _value, uint8_t _nb,
                                           uint32_t _timeout_us) {
  if (_nb > 4)
    return false;

  sdo_can_frame sdo_config_param = {.cob_ID =
                                        (uint16_t)(TPDO_COB_ID + controller_ID),
                                    .ccs = 1,
                                    .n = (uint8_t)(4 - _nb),
                                    .e = 1,
                                    .s = 1,
                                    .index = _index,
                                    .sub_index = _sub_index,
                                    .num_data_bytes = _nb};
  for (uint8_t _i = 0; _i < _nb; _i++) {
    sdo_config_param.data[_i] = (uint8_t)(_value >> (_i << 3));
  }

  if (!_can.write_SDO(&sdo_config_param))
    return false;
  if (!wait_SDO(_timeout_us))
    return false;
  if (sdo_received.index != _index)
    return false;
  if (sdo_received.sub_index != _sub_index)
    return false;
  if (sdo_received.ccs != 0x03)
    return false;

  return true;
}

bool Curtis_ACF2A_Can_Driver::read_param(uint16_t _index, uint8_t _sub_index,
                                         int32_t *_value,
                                         uint32_t _timeout_us) {
  *_value = 0;
  sdo_can_frame sdo_config_param = {.cob_ID =
                                        (uint16_t)(TPDO_COB_ID + controller_ID),
                                    .ccs = 2,
                                    .n = 0,
                                    .e = 0,
                                    .s = 0,
                                    .index = _index,
                                    .sub_index = _sub_index,
                                    .num_data_bytes = 0};

  if (!_can.write_SDO(&sdo_config_param))
    return false;
  if (!wait_SDO(_timeout_us))
    return false;
  if (sdo_received.index != _index)
    return false;
  if (sdo_received.sub_index != _sub_index)
    return false;
  if (sdo_received.ccs != 0x02)
    return false;

  for (uint8_t _i = 0; _i < sdo_received.num_data_bytes; _i++) {
    *_value |= sdo_received.data[_i] << (_i << 3);
  }

  return true;
}

bool Curtis_ACF2A_Can_Driver::wait_SDO(uint32_t _timeout_us) {
  sdo_received_flag = false;
  auto startTime = std::chrono::high_resolution_clock::now();
  while (!sdo_received_flag) {
    usleep(500);
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(
        currentTime - startTime);
    if (elapsedTime.count() >= _timeout_us)
      return false;
  }

  return true;
}

nmt_state Curtis_ACF2A_Can_Driver::get_state() { return (nmt_state)NMT_state; }

bool Curtis_ACF2A_Can_Driver::wait_state(nmt_state _nmt_state,
                                         uint16_t _timeout_ms) {
  nmt_received_flag = false;
  auto startTime = std::chrono::high_resolution_clock::now();
  while (!nmt_received_flag) {
    usleep(1000);
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(
        currentTime - startTime);
    if (elapsedTime.count() >= (_timeout_ms * 1000)) {
      std::cerr << "0x" << std::hex << (int)controller_ID
                << " Wait state timeout" << std::endl;
      return false;
    }
  }
  if (_nmt_state == BOOT_UP) {
    if ((get_state() == BOOT_UP) | (get_state() == PRE_OPERATIONAL))
      return true;
  } else if (get_state() == _nmt_state)
    return true;
  std::cerr << "0x" << std::hex << (int)controller_ID
            << " Failed to change motor state" << std::endl;
  return false;
}

int8_t
Curtis_ACF2A_Can_Driver::check_state_go_operational(uint16_t _timeout_ms) {
  if (motor_configured_flag) {
    if (NMT_state == PRE_OPERATIONAL) {
      if (set_state(GO_OPERATIONAL)) {
        wait_state(OPERATIONAL, _timeout_ms);
        return 1;
      } else
        return -1;
    } else
      return 0;
  } else
    return -1;
}

curtis_error_codes Curtis_ACF2A_Can_Driver::get_error_code() {
  return (curtis_error_codes)error_code;
}

const char *Curtis_ACF2A_Can_Driver::get_error_message() {
  switch (error_code) {
  case 0x00:
    return "No error";
  case 0x12:
    return "Controller Overcurrent";
  case 0x13:
    return "Current Sensor";
  case 0x14:
    return "Precharge Failed";
  case 0x15:
    return "Controller Severe Undertemperature";
  case 0x16:
    return "Controller Severe Overtemperature";
  case 0x17:
    return "Severe Undervoltage";
  case 0x18:
    return "Severe Overvoltage";
  case 0x19:
    return "Speed Limit Supervision";
  case 0x1A:
    return "Motor Not Stopped";
  case 0x1B:
    return "Critical OS General";
  case 0x1C:
    return "OS General 2";
  case 0x1E:
    return "Motor Short";
  case 0x1D:
    return "Reset Rejected";
  case 0x22:
    return "Controller Overtemperature Cutback";
  case 0x23:
    return "Undervoltage Cutback";
  case 0x24:
    return "Overvoltage Cutback";
  case 0x25:
    return "External 5V Supply Failure";
  case 0x26:
    return "External 12V Supply Failure";
  case 0x28:
    return "Motor Temperature Hot Cutback";
  case 0x29:
    return "Motor Temperature Sensor";
  case 0x31:
    return "Main Driver";
  case 0x32:
    return "EM Brake Driver";
  case 0x33:
    return "Pump Driver";
  case 0x34:
    return "Load Hold Driver";
  case 0x35:
    return "Lower Driver";
  case 0x36:
    return "Motor Feedback";
  case 0x37:
    return "Motor Open";
  case 0x38:
    return "Main Contactor Welded";
  case 0x39:
    return "Main Contactor Did Not Close";
  case 0x3A:
    return "Motor Setup Needed";
  case 0x42:
    return "Throttle Input";
  case 0x44:
    return "Brake Input";
  case 0x46:
    return "NV Memory Failure";
  case 0x47:
    return "Emergency HPD";
  case 0x49:
    return "Parameter Change";
  case 0x4A:
    return "EMR Switch Redundancy";
  case 0x51:
    return "User 1 Fault";
  case 0x52:
    return "User 2 Fault";
  case 0x53:
    return "User 3 Fault";
  case 0x54:
    return "User 4 Fault";
  case 0x55:
    return "User 5 Fault";
  case 0x56:
    return "User 6 Fault";
  case 0x57:
    return "User 7 Fault";
  case 0x58:
    return "User 8 Fault";
  case 0x59:
    return "User 9 Fault";
  case 0x61:
    return "User 10 Fault";
  case 0x62:
    return "User 11 Fault";
  case 0x63:
    return "User 12 Fault";
  case 0x64:
    return "User 13 Fault";
  case 0x65:
    return "User 14 Fault";
  case 0x66:
    return "User 15 Fault";
  case 0x67:
    return "User 16 Fault";
  case 0x5A:
    return "User 17 Fault";
  case 0x5B:
    return "User 18 Fault";
  case 0x5C:
    return "User 19 Fault";
  case 0x5D:
    return "User 20 Fault";
  case 0x5E:
    return "User 21 Fault";
  case 0x5F:
    return "User 22 Fault";
  case 0x6A:
    return "User 23 Fault";
  case 0x6B:
    return "User 24 Fault";
  case 0x6C:
    return "User 25 Fault";
  case 0x6D:
    return "User 26 Fault";
  case 0x6E:
    return "User 27 Fault";
  case 0x6F:
    return "User 28 Fault";
  case 0x68:
    return "VCL Run Time Error";
  case 0x71:
    return "OS General";
  case 0x72:
    return "PDO Timeout";
  case 0x73:
    return "Stall Detected";
  case 0x77:
    return "Supervision";
  case 0x79:
    return "Supervision Input Check";
  case 0x82:
    return "PDO Mapping Error";
  case 0x83:
    return "Internal Hardware";
  case 0x84:
    return "Motor Braking Impaired";
  case 0x87:
    return "Motor Characterization";
  case 0x88:
    return "Encoder Pulse Error";
  case 0x89:
    return "Parameter Out of Range";
  case 0x91:
    return "Bad Firmware";
  case 0x92:
    return "EM Brake Failed to Set";
  case 0x93:
    return "Encoder LOS";
  case 0x94:
    return "Emergency Reverse Timeout";
  case 0x96:
    return "Pump BDI";
  case 0x99:
    return "Parameter Mismatch";
  case 0x9A:
    return "Interlock Braking Supervision";
  case 0x9B:
    return "EMR Supervision";
  case 0xA1:
    return "Driver 1 Fault";
  case 0xA2:
    return "Driver 2 Fault";
  case 0xA3:
    return "Driver 3 Fault";
  case 0xA4:
    return "Driver 4 Fault";
  case 0xA5:
    return "Driver 5 Fault";
  case 0xA6:
    return "Driver 6 Fault";
  case 0xA7:
    return "Driver 7 Fault";
  case 0xA8:
    return "Driver Assignment";
  case 0xA9:
    return "Coil Supply";
  case 0xB1:
    return "Analog 1 Out of Range";
  case 0xB2:
    return "Analog 2 Out of Range";
  case 0xB3:
    return "Analog 3 Out of Range";
  case 0xB4:
    return "Analog 4 Out of Range";
  case 0xB5:
    return "Analog 5 Out of Range";
  case 0xB6:
    return "Analog 6 Out of Range";
  case 0xB7:
    return "Analog 7 Out of Range";
  case 0xB8:
    return "Analog 8 Out of Range";
  case 0xB9:
    return "Analog 9 Out of Range";
  case 0xBB:
    return "Analog 14 Out of Range";
  case 0xBD:
    return "Analog 18 Out of Range";
  case 0xBE:
    return "Analog 19 Out of Range";
  case 0xBC:
    return "Analog Assignment";
  case 0xC1:
    return "Branding Error";
  case 0xC2:
    return "BMS Cutback";
  case 0xC5:
    return "PWM Input 10 Out of Range";
  case 0xC7:
    return "Analog 31 Out of Range";
  case 0xC8:
    return "Invalid CAN Port";
  case 0xC9:
    return "VCL Watchdog";
  case 0xCB:
    return "PWM Input 28 Out of Range";
  case 0xD1:
    return "Lift Input";
  case 0xD2:
    return "Phase PWM Mismatch";
  case 0xD3:
    return "Hardware Compatibility";
  case 0xD4:
    return "Lower Input";
  case 0xD6:
    return "Hazardous Movement";
  case 0xDD:
    return "IMU Failure";
  case 0xFF:
    return "Not Initialized";
  default:
    return "Unknown Error Code";
  }
}

const char *Curtis_ACF2A_Can_Driver::get_state_message() {
  switch (NMT_state) {
  case BOOT_UP:
    return "BOOT_UP";
  case STOPPED:
    return "STOPPED";
  case OPERATIONAL:
    return "OPERATIONAL";
  case PRE_OPERATIONAL:
    return "PRE_OPERATIONAL";
  case COMM_TIMEOUT:
    return "COMM_TIMEOUT";
  default:
    return "Unknown state";
  }
}

bool Curtis_ACF2A_Can_Driver::set_state(nmt_commands _nmt_state) {
  return _can.write_NMT(_nmt_state, controller_ID);
}

#endif // CURTIS_ACF2A_CAN_DRIVER_LIB_CPP
