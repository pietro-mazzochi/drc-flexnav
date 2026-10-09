#!/bin/bash

GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m' # Sem cor

echo -e "${CYAN}Iniciando o Joystick...${NC}"
sleep 1
echo -e "${GREEN}Joystick configurado com sucesso!${NC}"
ros2 launch joystick_controller joystick_control.launch.xml
