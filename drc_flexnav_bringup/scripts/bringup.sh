#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m' # Sem cor

echo -e "${CYAN}Iniciando o setup CAN e o bringup do robô...${NC}"

# Função para configurar uma interface CAN
configure_can() {
    local interface=$1
    local bitrate=$2

    # Verifica se a interface já está configurada corretamente
    current_config=$(ip -details link show $interface | grep -o "bitrate $bitrate")
    if [[ $current_config == "bitrate $bitrate" ]]; then
        echo -e "${GREEN}${interface} já está configurada com bitrate ${bitrate}.${NC}"
    else
        echo -e "${YELLOW}Configurando ${interface} com bitrate ${bitrate}...${NC}"
        sudo ip link set $interface down
        sudo ip link set $interface up type can bitrate $bitrate
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}${interface} configurada com sucesso!${NC}"
        else
            echo -e "${RED}Erro ao configurar ${interface}! Verifique as permissões e a conexão.${NC}"
            exit 1
        fi
    fi
}

# Configuração das interfaces CAN
configure_can "can0" 250000
configure_can "can1" 250000

# Iniciando o launcher
echo -e "${CYAN}Iniciando o bringup do robô...${NC}"
ros2 launch drc_flexnav_bringup flexnav_bringup.launch.py
if [ $? -eq 0 ]; then
    echo -e "${GREEN}Bringup iniciado com sucesso!${NC}"
else
    echo -e "${RED}Erro ao iniciar o bringup! Verifique o arquivo de launch.${NC}"
    exit 1
fi
