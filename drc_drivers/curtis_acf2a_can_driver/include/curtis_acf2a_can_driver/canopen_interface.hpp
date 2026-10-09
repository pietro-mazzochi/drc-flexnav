#ifndef CANOPEN_INTERFACE
#define CANOPEN_INTERFACE

#include <cstdint>
#include <string>
#include <cstring>
#include <iostream>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

/**
*   @brief  Define FIFO Size
*/
#define     FIFO_SIZE       20

/**
*   @brief  Define CANOpen Function Codes
*/
#define     FC_NMT          0x00
#define     FC_SYNC         0x01
#define     FC_EMCY         0x01
#define     FC_TIME         0x02
#define     FC_TPDO1        0x03
#define     FC_RPDO1        0x04
#define     FC_TPDO2        0x05
#define     FC_RPDO2        0x06
#define     FC_TPDO3        0x07
#define     FC_RPDO3        0x08
#define     FC_TPDO4        0x09
#define     FC_RPDO4        0x0A
#define     FC_TSDO         0x0B
#define     FC_RSDO         0x0C
#define     FC_HEARTBEAT    0x0E

/**
*   @brief  Struct to CANOpen SDO frame
*/
struct sdo_can_frame
{
    uint16_t cob_ID;        // COB-ID. 0x600 + Controlled_ID for RPDO. 0x580 + Controlled_ID for TPDO.
    uint8_t ccs;            // 1: Download (PC -> Controller), 2: Upload (Controller -> PC)
    uint8_t n;              // #bytes in data bytes 4-7 that do not contain data (valid if e & s are set)
    uint8_t e;              // 0 or 1. Indicates an "expedited transfer" (all data is in a single CAN frame)
    uint8_t s;              // 0 or 1. Indicates that data size is shown in n
    uint16_t index;         // Reflect the OD address to be accessed
    uint8_t sub_index;      // Reflect the OD address to be accessed
    uint8_t data[4];        // bytes 4-7 contain the data
    uint8_t num_data_bytes; // Number of data bytes to send or received (1-4)
};

/**
*   @brief  Struct to CANOpen PDO frame
*/
struct pdo_can_frame
{
    uint16_t cob_ID;        // COB-ID
    uint8_t data_size;      // Number of bytes
    uint8_t data[8];        // Data bytes
};

/**
*   @brief  Enum to CANOpen NMT state code
*/
enum nmt_state
{
    BOOT_UP             = 0x00,
    STOPPED             = 0x04,   
    OPERATIONAL         = 0x05,
    PRE_OPERATIONAL     = 0x7F,
    COMM_TIMEOUT        = 0xFF
};

/**
*   @brief  Enum to CANOpen NMT commands
*/
enum nmt_commands
{
    GO_OPERATIONAL         = 0x01,
    GO_STOPPED             = 0x02,    
    GO_PRE_OPERATIONAL     = 0x80,
    GO_RESET_NODE          = 0x81,
    GO_RESET_COMMUNICATION = 0x82
};

/**
 *   @brief  CAN_Interface Class
 */
class CANOPEN_Interface
{
public:
    CANOPEN_Interface(void);

    /**
     *
     * @brief   Function to disconnect the CAN and clean up the context
     *
     */
    ~CANOPEN_Interface(void);

    /**
     *
     * @brief   Function to initialize the CAN interface
     *
     * @param   can_interface   CAN interface name
     *
     */
    bool init(const std::string &can_interface);

    /**
     *
     * @brief   Function to disconnect the CAN and clean up the context
     *
     */
    bool deinit(void);

    /**
     *
     * @brief   Function to get Socket CAN number
     *
     * @return  Socket CAN number
     *
     */


    int get_can_socket(void);

    /**
     *
     * @brief   Function to write CANOpen SDO (Receive SDO)
     *
     * @param   _sdo_frame      Structure "sdo_can_frame" with the data read
     * @param   extended_frame  CAN use Extended Frame Format
     *
     * @return  Success / Fail
     *
     */
    bool write_SDO(sdo_can_frame *_sdo_frame, bool extended_frame = false);

    /**
     *
     * @brief   Function to write CANOpen SDO (Receive SDO)
     *
     * @param   _nmt_command    NMT command
     * @param   _node_ID        Node ID to send command
     *
     * @return  Success / Fail
     *
     */
    bool write_NMT(nmt_commands _nmt_command, uint8_t _node_ID, bool extended_frame = false);

    /**
     *
     * @brief   Function to write CANOpen RPDO
     *
     * @param   _write_pdo    Pointer to struct "pdo_can_frame"
     *
     * @return  Success / Fail
     *
     */
    bool write_RPDO(pdo_can_frame *_write_pdo, bool extended_frame = false);

    /**
     *
     * @brief   Loop function that parses the received CAN data to FIFO
     * 
     * @param   read_timeout_us    Read timeout in us ( 0 - 999999 )
     *
     */
    bool read_loop(uint32_t read_timeout_us = 200000);

    /**
     *
     * @brief   Function that parses the received CAN data to FIFO
     *
     * @return  Success / Fail
     */
    bool read_fifo(uint8_t _read_ID, can_frame *_read_fifo_frame);

private:
    /**
     *
     * @brief   CAN context variables
     *
     */
    int can_socket;
    struct sockaddr_can addr;
    struct ifreq ifr;
    can_frame buffer_fifo[128][FIFO_SIZE] = {};
    uint8_t buffer_head[128];
    uint8_t buffer_tail[128];
};

#endif // CANOPEN_INTERFACE
