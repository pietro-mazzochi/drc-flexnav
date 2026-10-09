# Nó para comunicação com driver de motor Curtis

Implementada stack de comunicação com driver de acionamneto da Curtis via CAN.  
Testado com o Curtis AC F2-A.

## Sumário

1. [Introdução](#introdução)
2. [Configuração da Interface CAN](#configuração-da-interface-can)
3. [Configuração do Baud Rate do Driver](#configuração-do-baud-rate-do-driver)
4. [Biblioteca `canopen_interface.hpp`](#biblioteca-canopen_interfacehpp)
   - Estrutura da classe CANOpen
   - Funções para leitura e escrita de dados CANOpen
5. [Biblioteca `curtis_motor_lib.cpp`](#biblioteca-curtis_motor_libcpp)
   - Estrutura da classe CURTIS_Motor
   - Funções para controle do driver de motor Curtis
6. [Configuração de TPDOs](#configurações-de-tpdos)
   - Estrutura para configurar e vincular dados de TPDOs
   - Exemplo de configuração de TPDO para encoder e RPM
7. [Configuração de RPDOs](#configurações-de-rpdos)
   - Estrutura para configurar e enviar dados de RPDOs
   - Exemplo de configuração de RPDO para comando de acelerador
8. [Configuração de parâmetros](#configuração-de-parâmetros)
9. [Estado NMT do driver](#estado-nmt-do-driver)
10. [Códigos de falha](#códigos-de-falha)

<br>

## Configuração da interface CAN

Inicialização e configuração da interface CAN para 1Mb/s (can0 é o nome da interface utilizada):
```bash
sudo ip link set can0 up type can bitrate 1000000
```  

<br>

## Configuração do baud rate do Driver

Para verificar a comunicação do driver pode ser utilizado o pacote `can-utils` por meio do comando `candump`.  
Caso o driver não esteja enviando dados (heartbeat) é possível que o baud rate não esteja configurado corretamente.  
Neste caso é necessário desativar a interface CAN e reativar com o baud rate atual do driver:
```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate <baud_rate_atual>
```  

Após inicializar a interface CAN deve ser enviado o comando (utilizando `can-utils`) para alteração do baud rate do driver para 1Mb/s:
```bash
cansend can0 6<id>#2308200001000000     # Enable saving of the changed data to flash memory
cansend can0 6<id>#2B0120010400         # Change the baud rate to 1 Mb/s
cansend can0 6<id>#2308200000000000     # Disable saving of the changed data to flash memory
cansend can0 000#81<id>                 # Reset node
``` 

ATENÇÃO: `<id>` corresponde ao ID do driver em hexadecimal. Exemplo para ID `0x26` (default):
```bash
cansend can0 626#2308200001000000     # Enable saving of the changed data to flash memory
cansend can0 626#2B0120010400         # Change the baud rate to 1 Mb/s
cansend can0 626#2308200000000000     # Disable saving of the changed data to flash memory
cansend can0 000#8126                 # Reset node
``` 

A interface CAN deve ser inicializada novamente em 1Mb/s:
```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```  

<br>

## Biblioteca `canopen_interface.hpp`

Contém as estruturas, definições de dados e a classe de interface CANOpen. Implementa um stack simples de comunicação CANOpen com os seguintes métodos:
```c
/**
*   @brief  CANOPEN_Interface Class
*/
class CANOPEN_Interface {
    public:
        /**
         * @brief   Function to initialize the CAN interface
         */
        bool init(const std::string& can_interface);

        /** 
         * @brief   Function to get Socket CAN number
         */
        int get_can_socket();

        /** 
         * @brief   Function to write CANOpen SDO (Receive SDO)
         */
        bool write_SDO(sdo_can_frame* _sdo_frame, bool extended_frame = false);

        /** 
         * @brief   Function to write CANOpen SDO (Receive SDO)  
         */
        bool write_NMT(nmt_commands _nmt_command, uint8_t _node_ID, bool extended_frame = false);

        /** 
         * @brief   Function to write CANOpen RPDO 
         */
        bool write_RPDO(pdo_can_frame* _write_pdo, bool extended_frame = false);

        /** 
         * @brief   Loop function that parses the received CAN data to FIFO
         */
        void read_loop();

        /** 
         * @brief   Function that parses the received CAN data to FIFO
         */
        bool read_fifo(uint8_t _read_ID, can_frame* _read_fifo_frame);

    private:
        /**
         * @brief   CAN context variables
         */
        int can_socket;
        struct sockaddr_can addr;
        struct ifreq ifr;
        /**
         * @brief   CAN FIFO variables to receive data
         */
        can_frame buffer_fifo[128][FIFO_SIZE];
        uint8_t buffer_head[128];
        uint8_t buffer_tail[128];
};
```

Para realizar a leitura de dados CAN é necessário chamar a função `read_loop()` de forma contínua.  
Esta função executa a leitura de dados do Socket, filtra pelo ID do device e alimenta a variável `can_frame buffer_fifo[128][FIFO_SIZE]` deixando os dados prontos para serem consumidos.  
O consumo dos dados é realizado pela função `read_fifo(uint8_t _read_ID, can_frame* _read_fifo_frame)`, que lê um frame CAN do FIFO do ID específico.  
 Na implementação realizada para o driver da Curtis, `read_fifo()` é chamada dentro da biblioteca do motor junto com a função `parse_loop()`.

<br>

## Biblioteca `curtis_motor_lib.cpp` 

Contém a classe de interface com o driver da Curtis. Implementa um stack com os seguintes métodos:
```c
/**
*   @brief  CURTIS_Motor Class
*/
class CURTIS_Motor {
    private:
        /** 
         * @brief   CAN context variables
         */
        CANOPEN_Interface & _can;
        uint8_t controller_ID;

        /** 
         * @brief   CAN TPDO config variables
         */
        pdo_config_t tpdo_cfg[4];

        /** 
         * @brief   CAN RPDO config variables
         */
        pdo_config_t rpdo_cfg[4];

        /** 
         * @brief   CAN TSDO variables
         */
        bool sdo_received_flag;
        bool nmt_received_flag;
        sdo_can_frame sdo_received;

        /** 
         * @brief   Motor variables
         */
        uint8_t NMT_state;
        uint8_t error_code;

    public:
        /**
         * @brief   Constructor for CURTIS_Motor class
         */
        CURTIS_Motor(CANOPEN_Interface& _can_interface);

        /** 
         * @brief   Function to initialize the interface for Curtis Motor controller
         */
        void init(uint8_t _controller_ID);

        /** 
         * @brief   Loop function that parses the received CAN data
         */
        bool parse_loop();

        /** 
         * @brief   Function to config TPDO of motor
         */
        bool config_TPDO(uint8_t _tpdo_number, pdo_config_t* _tpdo_cfg, uint32_t _timeout_us = 10000);

        /** 
         * @brief   Function to config RPDO of motor
         */
        bool config_RPDO(uint8_t _rpdo_number, pdo_config_t* _rpdo_cfg, uint32_t _timeout_us = 10000);

        /** 
         * @brief   Function to write RPDO to motor
         */
        bool write_RPDO(uint8_t _rpdo_number);

        /** 
         * @brief   Function to config motor parameter using SDO
         */
        bool config_param(uint16_t _index, uint8_t _sub_index, int32_t _value, uint8_t _nb, uint32_t _timeout_us = 10000);

         /** 
         * @brief   Function to wait CANOpen SDO (Transmit SDO)
         */
        bool wait_SDO(uint32_t _timeout_us);

        /** 
         * @brief   Function to get NMT State of motor
         */
        nmt_state get_state();

        /** 
         * @brief   Function to wait NMT State of motor
         */
        bool wait_state(nmt_state _nmt_state, uint16_t _timeout_ms = 100);

        /** 
         * @brief   Function to get error code of motor
         */
        curtis_error_codes get_error_code();

        /** 
         * @brief   Function to get error message of motor
         */
        const char* get_error_message(curtis_error_codes _error_code);

        /** 
         * @brief   Function to send NMT command to motor
         */
        bool set_state(nmt_commands _nmt_state);
};
```

Para realizar a leitura de dados dor motores é necessário chamar a função `parse_loop()` de forma contínua.  
Esta função executa a leitura de dados do FIFO e realiza o parse destes dados para as variáveis específicas.  

<br>

## Configurações de TPDOs

Para os TPDOs foram implementados métodos que realizam tanto a configuração do motor para os dados especificados quanto a vinculação destes dados com variáveis por meio de ponteiros.  
A configuração de TPDO é realizada pela função `config_TPDO()` com base na estrutura:

```c
/**
*   @brief  Struct to config PDOs
*/
struct pdo_config_t {
    uint8_t count;                 // Number of present data (1 - 8)
    uint8_t sizes[8];              // Number of bytes for each data (1 - 4)
    uint16_t index[8];             // Index of each PDO data
    uint8_t sub_index[8];          // Sub-Index of each PDO data
    uint16_t timeout_ms;           // Time interval to controller send data (ms)
    void* data_ptr[8];             // Pointer to data
};
```

Exemplo de configuração da TPDO1 para entregar os dados de contegem de encoder na variável `motor1_encoder` e a velocidade de rotação em RPM na variável `motor1_rpm`.  
ATENÇÃO: Estas variáveis são atualizadas na função `parse_loop()`, por isso é importante que a função `parse_loop()` seja chamada de forma contínua com intervalo menor ou igual a taxa de saída do TPDO.  
O intervalo de saída dos dados é configurado na variável `timeout_ms` da estrutura. Para o Driver AC F2-A o intervalo mínimo é de 4 ms (mesmo configurando um valor menor do que 4, os dados serão entregues em 4 ms).  
Para desativar um TPDO basta configurar a variável `count` da estrutura para 0 (zero).

```c
// Config motors TPDO1
pdo_config_t tpdo1_cfg = {
    .count = 2
};
tpdo1_cfg.sizes[0] = sizeof(motor1_encoder);
tpdo1_cfg.data_ptr[0] = &motor1_encoder;
tpdo1_cfg.index[0] = 0x3523;
tpdo1_cfg.sub_index[0] = 0x00;
tpdo1_cfg.sizes[1] = sizeof(motor1_rpm);
tpdo1_cfg.data_ptr[1] = &motor1_rpm;
tpdo1_cfg.index[1] = 0x352F;
tpdo1_cfg.sub_index[1] = 0x00;
tpdo1_cfg.timeout_ms = 0;        // Minimum time interval on Curtis is 4ms (0 = 4ms)
if(motor1.config_TPDO(1, &tpdo1_cfg))       RCLCPP_INFO(this->get_logger(),"Motor 1 TPDO1 config success!");
else                                        RCLCPP_INFO(this->get_logger(),"Motor 1 TPDO1 config fail!");
```

<br>

## Configurações de RPDOs

Para os RPDOs foram implementados métodos que realizam tanto a configuração do motor para os dados especificados quanto a vinculação destes dados com variáveis por meio de ponteiros.  
A configuração de RPDO é realizada pela função `config_RPDO()` com base na estrutura:

```c
/**
*   @brief  Struct to config PDOs
*/
struct pdo_config_t {
    uint8_t count;                 // Number of present data (1 - 8)
    uint8_t sizes[8];              // Number of bytes for each data (1 - 4)
    uint16_t index[8];             // Index of each PDO data
    uint8_t sub_index[8];          // Sub-Index of each PDO data
    uint16_t timeout_ms;           // Time interval to controller send data (ms)
    void* data_ptr[8];             // Pointer to data
};
```

Exemplo de configuração da RPDO1 para enviar comandos de acelerador ao driver contidos na variável `motor1_throttle_`. O RPDO 1 está, neste exemplo, vinculado a variável `motor1_throttle_`.  
ATENÇÃO: O envio do RPDO ocorre com a chamada da função `write_RPDO(_rpdo_number)`, ou seja, quando chamada `write_RPDO(1)` ocorre o envio do RPDO1 ao driver.  
Os RPDOs possuem um timeout que é configurado na variável `timeout_ms` da estrutura. Isso significa que o RPDO devce ser enviado de forma cíclica em período menor que o timeout configurado, caso contrário o driver entra em falha.  
Para desativar um RPDO basta configurar a variável `count` da estrutura para 0 (zero).

```c
// Config motor 1 RPDO1
pdo_config_t rpdo1_cfg = {
    .count = 1
};
rpdo1_cfg.sizes[0] = sizeof(motor1_throttle_);
rpdo1_cfg.data_ptr[0] = &motor1_throttle_;
rpdo1_cfg.index[0] = 0x3366;
rpdo1_cfg.sub_index[0] = 0x00;
tpdo1_cfg.timeout_ms = 40;        // 40ms
if(motor1.config_RPDO(1, &rpdo1_cfg))       RCLCPP_INFO(this->get_logger(),"Motor 1 RPDO1 config success!");
else                                        RCLCPP_INFO(this->get_logger(),"Motor 1 RPDO1 config fail!");
```

<br>

## Configuração de parâmetros

Para realizar a configuração de algum parâmetro do driver foi implementada a função `config_param()` que utiliza transações SDO para realizar esta configuração.  
Esta função envia o comando de configuração e aguarda a resposta do driver. É retornado `true` apenas se o driver confirmar a alteração de parâmetro.  
Exemplo de configuração do controle para o modo de torque:

```c
// Config motor parameters
uint16_t control_mode = 2;          // 0 = Speed Mode Express; 1 = Speed Mode; 2 = Torque Mode
if(motor1.config_param(0x3080, 0x00, control_mode, sizeof(control_mode)))   RCLCPP_INFO(this->get_logger(),"Motor 1 Control Mode config success!");
else                                                                        RCLCPP_INFO(this->get_logger(),"Motor 1 Control Mode config fail!");
```

<br>

## Estado NMT do driver

O driver envia o estado NMT no heartbeat. O estado atual pode ser obtido pela função `get_state()`. Os estados possíveis são:

```c
/**
*   @brief  Enum to CANOpen NMT state code
*/
enum nmt_state
{
    BOOT_UP             = 0x00,
    STOPPED             = 0x04,   
    OPERATIONAL         = 0x05,
    PRE_OPERATIONAL     = 0x7F
};
```

Quando energizado o driver inicia com estado `PRE_OPERATIONAL`.  
Para alterar o estado basta utilizar a função `set_state()`. Os comandos possíveis são:

```c
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
```

Para recuperar o driver de algum estado de falha é necessário enviar um comando de reset ao nó: `set_state(GO_RESET_NODE)`.  
Após o reset o driver retorna em estado `PRE_OPERATIONAL`.

<br>

## Códigos de falha

O driver possui inúmeros códigos de falha que são obtidos por meio da função `get_error_code()`. Em estado normal de operação o retorno é 0 (zero).