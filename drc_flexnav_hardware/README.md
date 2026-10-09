# drc_flexnav_hardware

Este diretório contém as interfaces de hardware (hardware interfaces) ROS2 para o robô AMR FlexNav. Cada interface implementa a comunicação entre o ROS2 e diferentes componentes de hardware do robô, utilizando o framework `ros2_control`.

## Interfaces de Hardware Presentes

### 1. Curtis ACF2A CAN Hardware Component
- **Arquivo:** `curtis_acf2a_can_hardware_component.cpp` / `curtis_acf2a_can_hardware_component.xml`
- **Classe:** `drc_flexnav_hardware::Curtis_ACF2A_Can_HW_Component`
- **Tipo:** `hardware_interface::SystemInterface`
- **Descrição:** Interface ros2_control para comunicação com o driver de motor Curtis ACF2A via barramento CAN.

### 2. IMU LPMS-CU3 Hardware Component
- **Arquivo:** `imu_lpms_cu3_hardware_component.cpp` / `imu_lpms_cu3_hardware_component.xml`
- **Classe:** `drc_flexnav_hardware::ImuLpmsCu3HardwareComponent`
- **Tipo:** `hardware_interface::SensorInterface`
- **Descrição:** Interface ros2_control para integração do sensor inercial LPMS-CU3 (IMU).

### 3. Siemens ET200SP Modbus Hardware Component
- **Arquivo:** `siemens_et200sp_modbus_hardware_component.cpp` / `siemens_et200sp_modbus_hardware_component.xml`
- **Classe:** `drc_flexnav_hardware::Siemens_ET200SP_Modbus_HW_Component`
- **Tipo:** `hardware_interface::SystemInterface`
- **Descrição:** Interface ros2_control para comunicação com o CLP Siemens ET200SP via protocolo Modbus.

## Estrutura dos Arquivos
- Os arquivos `*.cpp` implementam a lógica de comunicação e integração com o hardware.
- Os arquivos `*.xml` registram as classes como plugins do ros2_control.

## Referências
- [ros2_control documentation](https://control.ros.org/)
- Documentação específica de cada componente pode ser encontrada nos respectivos arquivos fonte e XML.  

## Alterações na Interface Siemens ET200SP - Consultoria

Na consultoria, após a finalização do projeto, foram implementadas as seguintes alterações na interface de hardware da Siemens ET200SP:

- **Leitura e publicação de tópicos diretamente na hardware interface**:
  - Implementação de publishers e subscribers ROS2 para comunicação direta com tópicos como `to_plc_command`, `to_plc_mission` e `plc_request`.
  - Publicação de mensagens do tipo `PlcRequest` para sinalizar comandos e missões ao PLC, incluindo comandos como `shutdown`, `mission_start` e `mission_cancel`.
  - Subscrição a tópicos para receber comandos e respostas de missão do PLC, processando mensagens do tipo `UInt8` e `PlcMissionResponse`.
- **Publicação de estados e diagnósticos**:
  - A interface passou a publicar informações detalhadas do estado do robô, sensores, atuadores e status de conexão com o PLC, facilitando o monitoramento e a integração com controladores ROS2.
  - Os estados publicados incluem informações como: conexão com o PLC, tempo da última leitura/escrita, comandos recebidos, missões, sensores de carga, erros e status de emergência.
- **Execução do nó ROS2 em thread dedicada**:
  - O nó interno da interface de hardware (`hw_node_`) passou a rodar em uma thread própria, garantindo responsividade na comunicação com o PLC e nos tópicos ROS2.

Essas alterações aumentaram a integração entre o ROS2 e o CLP Siemens, viabilizando o fluxo de dados para criação de missões pelo CLP e feedback do estado atual das missões ao CLP.
