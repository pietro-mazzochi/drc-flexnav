#include "curtis_acf2a_can_driver/canopen_interface.hpp"

CANOPEN_Interface::CANOPEN_Interface(void)
{
}

bool CANOPEN_Interface::init(const std::string &can_interface)
{
    // Create CAN socket
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket < 0)
    {
        std::cerr << "Error creating CAN socket!" << std::endl;
        return false;
    }

    // Get the network interface index
    strcpy(ifr.ifr_name, can_interface.c_str());
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0)
    {
        std::cerr << "Error getting CAN interface index!" << std::endl;
        close(can_socket);
        can_socket = -1;
        return false;
    }

    // Configure address for the CAN interface
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Bind the socket to the CAN interface
    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        std::cerr << "Error binding CAN socket to interface!" << std::endl;
        close(can_socket);
        can_socket = -1;
        return false;
    }

    // Set the socket to non-blocking mode
    int flags = fcntl(can_socket, F_GETFL, 0);
    if (flags == -1)
    {
        std::cerr << "Error retrieving socket flags: " << strerror(errno) << std::endl;
        return false;
    }
    if (fcntl(can_socket, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        std::cerr << "Error setting the socket to non-blocking mode: " << strerror(errno) << std::endl;
        return false;
    }

    //struct timeval timeout;
    //timeout.tv_sec = 0;
    //timeout.tv_usec = read_timeout_us;

    //if (setsockopt(can_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
    //    std::cerr << "CAN interface timeout config error." << std::endl;
    //    close(can_socket);
    //    can_socket = -1;
    //    return false;
    //}

    std::cout << "CAN interface " << can_interface << " successfully configured." << std::endl;

    for (uint8_t count = 0; count < 128; count++)
    {
        buffer_head[count] = 0;
        buffer_tail[count] = 0;
    }

    return true;
}

bool CANOPEN_Interface::deinit() {
    if (can_socket >= 0) {
	close(can_socket);  // Close the socket properly
	std::cout << "CAN socket closed." << std::endl;
	return true;
    }
    return false;
}


CANOPEN_Interface::~CANOPEN_Interface() {
  deinit();
}

int CANOPEN_Interface::get_can_socket(void)
{
    return can_socket;
}

bool CANOPEN_Interface::write_SDO(sdo_can_frame *_sdo_frame, bool extended_frame)
{
    if (_sdo_frame->num_data_bytes > 4)
        return false;

    if ((_sdo_frame->cob_ID & 0x780) != 0x600)
        return false;
    if (_sdo_frame->ccs > 6)
        return false;
    if (_sdo_frame->n > 3)
        return false;
    if (_sdo_frame->e > 1)
        return false;
    if (_sdo_frame->s > 1)
        return false;

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));

    if (extended_frame)
        frame.can_id = _sdo_frame->cob_ID | CAN_EFF_FLAG;
    else
        frame.can_id = _sdo_frame->cob_ID;
    frame.can_dlc = 4 + _sdo_frame->num_data_bytes;

    frame.data[0] = (_sdo_frame->ccs << 5) + (_sdo_frame->n << 2) + (_sdo_frame->e << 1) + (_sdo_frame->s);
    frame.data[1] = (_sdo_frame->index & 0xFF);
    frame.data[2] = (_sdo_frame->index >> 8);
    frame.data[3] = _sdo_frame->sub_index;

    for (uint8_t _i = 0; _i < _sdo_frame->num_data_bytes; _i++)
    {
        frame.data[4 + _i] = _sdo_frame->data[_i];
    }

    if (write(can_socket, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame))
        return true;

    return false;
}

bool CANOPEN_Interface::write_NMT(nmt_commands _nmt_command, uint8_t _node_ID, bool extended_frame)
{
    if (_node_ID < 1 || _node_ID > 127)
        return false;

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));

    if (extended_frame)
        frame.can_id = 0x00 | CAN_EFF_FLAG;
    else
        frame.can_id = 0x00;
    frame.can_dlc = 2;
    frame.data[0] = (uint8_t)_nmt_command;
    frame.data[1] = _node_ID;

    if (write(can_socket, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame))
        return true;

    return false;
}

bool CANOPEN_Interface::write_RPDO(pdo_can_frame *_write_pdo, bool extended_frame)
{
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));

    if (extended_frame)
        frame.can_id = _write_pdo->cob_ID | CAN_EFF_FLAG;
    else
        frame.can_id = _write_pdo->cob_ID;
    frame.can_dlc = _write_pdo->data_size;

    memcpy(frame.data, _write_pdo->data, frame.can_dlc);

    if (write(can_socket, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame))
        return true;

    return false;
}

bool CANOPEN_Interface::read_loop(uint32_t read_timeout_us)
{
    can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));
    int ret;
    fd_set rfds;

    FD_ZERO(&rfds);
    FD_SET(can_socket, &rfds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = read_timeout_us;

    ret = select(can_socket + 1, &rfds, NULL, NULL, &timeout);
        
    if(ret > 0){
        int nbytes = read(can_socket, &frame, sizeof(struct can_frame));
        if (nbytes != sizeof(struct can_frame))       return false;

        int _deviceId = frame.can_id & 0x7F; // Device ID
        if (_deviceId > 0 && _deviceId < 128)
        {
            memcpy(&buffer_fifo[_deviceId][buffer_head[_deviceId]], &frame, sizeof(frame));
            buffer_head[_deviceId] = (buffer_head[_deviceId] + 1) % FIFO_SIZE;

            return true;
        }
    }

    return false;
}

bool CANOPEN_Interface::read_fifo(uint8_t _read_ID, can_frame *_read_fifo_frame)
{
    if (_read_ID < 1 || _read_ID > 127)
        return false;

    if (buffer_tail[_read_ID] != buffer_head[_read_ID])
    {
        memcpy(_read_fifo_frame, &buffer_fifo[_read_ID][buffer_tail[_read_ID]], sizeof(can_frame));
        buffer_tail[_read_ID] = (buffer_tail[_read_ID] + 1) % FIFO_SIZE;
        return true;
    }

    return false;
}
