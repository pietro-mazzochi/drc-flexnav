
# DALCA ROBOT PLC

LIB TO EXCHANGE DATA WITH DALCA ROBOT PLC
VERSION 2.0 (added MESSAGES, COMMANDS and MISSION control on 17 jul 2025)

## DATA MAP ON PLC:

Program blocks -> new data block -> MB_DB (SIEMENS TIA PORTAL BASED)

<span style="color:lightGreen">

|Name  |  |          |Data type  | byte offset|Start value| Used for    |Note|
|------|--|------    |-----------|-------|-----------|----------------------|---|          
|HOLD_REG           |||Struct    |       |           |                      |   | 
||LEFT_ENC           ||DInt      |   0   |     0     | Left Wheel Encoder   | 7 | 
||RIGHT_ENC	         ||DInt      |   4   |     0     | Right Wheel Encoder  | 7 | 
||TIMESTAMP	         ||UDInt     |   8   |     0     | ms count timestamp   | 8 | 
||JOY_UP	            ||SInt      |  12   |     0     |  \|                  |   | 
||JOY_DOWN	         ||SInt      |  13   |     0     |  \|  Joystick        | 9 | 
||JOY_LEFT	         ||SInt      |  14   |     0     |  \|                  |   | 
||JOY_RIGHT	         ||SInt      |  15   |     0     |  \|                  |   | 
||DI                 ||Struct    |  16   |           |                      |   |   
|||LS_FRONT_BIT0      |Bool      |  16.0 |   false   |  \|                  |   | 
|||LS_FRONT_BIT1      |Bool      |  16.1 |   false   |  \|                  |   | 
|||LS_FRONT_BIT2      |Bool      |  16.2 |   false   |  \|  Front and Rear  |   | 
|||LS_FRONT_BIT3      |Bool      |  16.3 |   false   |  \|  laser scanner   |   | 
|||LS_REAR_BIT0       |Bool      |  16.4 |   false   |  \|  status          |   | 
|||LS_REAR_BIT1       |Bool      |  16.5 |   false   |  \|                  |   | 
|||LS_REAR_BIT2       |Bool      |  16.6 |   false   |  \|                  |   | 
|||LS_REAR_BIT3       |Bool      |  16.7 |   false   |  \|                  |   | 
|||AUTO_SW            |Bool      |  17.0 |   false   | Automatic SW status  |   | 
|||MAN_SW             |Bool      |  17.1 |   false   | Manual SW status     |   | 
|||RESET_SW           |Bool      |  17.2 |   false   | Reset button status  |   | 
|||TABLE_FRONT_LOAD   |Bool      |  17.3 |   false   | Table front sensor   |   | 
|||TABLE_REAR_LOAD    |Bool      |  17.4 |   false   | Table rear sensor    |   | 
|||JOYSTICK_CONNECTED |Bool      |  17.5 |   false   | Joystick connected   |   | 
|||LOADED             |Bool      |  17.6 |   false   | Load status (load on)|   | 
|||CHARGING           |Bool      |  17.7 |   false   | Battery charging(TBI)|   | 
||ERROR_CODE         ||Struct    |  18   |           |                      |   | 
|||RESET_WAIT         |Bool      |  18.0 |   false   | PLC waiting RST      |   | 
|||UNDEFINED          |Bool      |  18.1 |   false   |                      |   | 
|||INITIAL_STATUS     |Bool      |  18.2 |   true    | Power start-up flag  |   | 
|||LEFT_MOTOR         |Bool      |  18.3 |   false   | Left Motor Error     |   | 
|||RIGHT_MOTOR        |Bool      |  18.4 |   false   | Right Motor Error    |   | 
|||TABLE              |Bool      |  18.5 |   false   | Table Motor Error    |   | 
|||LOAD               |Bool      |  18.6 |   false   | Load (timeout) Error |   | 
|||UNLOAD             |Bool      |  18.7 |   false   | Unload (tmout) Error |   | 
|||EMERGENCY          |Bool      |  19.0 |   false   | Emergency Error      |   | 
|||BATTERY            |Bool      |  19.1 |   false   | Battery Error (TBI)  |   | 
|||DOCKING            |Bool      |  19.2 |   false   | Docking Error (TBI)  |   | 
|||MAINTANCE          |Bool      |  19.3 |   false   | Maintance Error (TBI)|   | 
|||UNUSED0            |Bool      |  19.4 |   false   |                      |   | 
|||UNUSED1            |Bool      |  19.5 |   false   |                      |   | 
|||UNUSED2            |Bool      |  19.6 |   false   |                      |   | 
|||UNUSED3            |Bool      |  19.7 |   false   |                      |   | 
||SEND_COMMAND       ||Byte      |  20   |     0     | Command PLC -> PC    | 1 | 
||RECEIVED           ||Byte      |  21   |           | Comnd extension(TBI) |   | 
||MISSION_TYPE       ||Byte      |  22   |     0     | Type of mission      | 2 |
||MISSION_SEQ        ||Byte      |  23   |     0     | Auto inc on new miss.|   |
||MISSION_LABEL      ||Array[0..3] of Char| 24 |     | Label of dest. NODE  |   |
||avoid              ||Array[0..13] of int| 28 |     |                      |   | 
||FRONT_RED_LIGHTS   ||Byte      |  56   |     0     | \|                   |   | 
||REAR_RED_LIGHTS    ||Byte      |  57   |     0     | \|                   |   | 
||FRONT_GREEN_LIGHTS ||Byte      |  58   |     0     | \| Visual and        |   | 
||REAR_GREEN_LIGHTS  ||Byte      |  59   |     0     | \| audible           | 3 | 
||LEFT_YELLOW_LIGHTS ||Byte      |  60   |     0     | \| signaling         |   | 
||RIGHT_YELLOW_LIGHTS||Byte      |  61   |     0     | \|                   |   | 
||BUZZER             ||Byte      |  62   |     0     | \|                   |   | 
||LOAD_UNLOAD_TIMEOUT||Byte      |  63   |     0     | Timeout for table    |10 | 
||LS_SELECT          ||Byte      |  64   |     0     | Laser Scanner Areas  | 4 | 
||UNUSED1            ||Byte      |  65   |     0     |                      |   | 
||COMMAND            ||Byte      |  66   |     0     | PC -> PLC COMMAND    | 1 | 
||RESERVED           ||Byte      |  67   |     0     |                      |   | 
||MISSION_FB_SEQ     ||Byte      |  68   |     0     | Incr on new FeedBack |   |
||MISSION_FB_STATUS  ||Byte      |  69   |     0     | Current Miss. Status | 5 |
||MESSAGE_TP         ||Byte      |  70   |     0     | Type of displ.message| 6 |
||MESSAGE_COD        ||Byte      |  71   |     0     | Numeric message cod. |   |
||MESSAGE_TXT        ||Array[0..19] of Char| 72 | -  | Msg String (to HMI)  |   |
||others             ||Array[0..89] of Byte| 92 | -  | For future expansions|   |

</span>

<br>

## notes:

### 1: command enumeration (same to send and receive):
```c
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
```
### 2: type of mission:<br>
   mission_type bit 0: 1 to cancel, 0 to start<br>
   mission_type bit 1: direction 0 to front, 1 to rear<br>
   mission_type bit 5: to enable unload service at dest<br>
   mission_type bit 6: to enable load service at dest<br>
   mission_type bit 7: to enable only move to dest (if enabled, bits 5 and 6 may be ignored)<br>

```c
typedef enum {
     MISSION_TYPE_MOVE      = (0b100 << 5),   // NEW boolean to set Move-only Mission Type
     MISSION_TYPE_LOAD      = (0b010 << 5),   // NEW boolean to set load at dest mission type 
     MISSION_TYPE_UNLOAD    = (0b001 << 5),   // NEW boolean to set unload at dest mission type
     MISSION_TYPE_FROM_REAR = (0b10),         // NEW boolean to set "dock from rear" at dest. When unset, dock from front
     MISSION_TYPE_CANCEL    = (0b01),         // NEW boolean to send a "cancel" command to mission
}    mission_command_tp;
```

### 3: visual and audible signaling:<br>

0 : off<br>
1 : continous on<br>
2 : fast blink (2Hz)<br>
3 : blink (1Hz)<br>
4 : slow blink (0.5Hz)<br>
255: controlled by PLC. To be implemented on PLC.

```c
typedef enum {
  SIGNALING_OFF = 0,
  SIGNALING_ON = 1,
  SIGNALING_FAST_BLINK = 2,
  SIGNALING_BLINK = 3,
  SIGNALING_SLOW_BLINK = 4,
  SIGNALING_BY_PLC = 255
} robot_signaling_status_t;
```
### 4: Laser Scanner Areas

| Tipo de seleção         |                            | C | D | E | F |  Output     |
|-------------------------|----------------------------|---|---|---|---|-------------|
|                         |          bits on Front LS  |0 1|2 3|4 5|6 7|  bits on    |
|                         |           bits on Rear LS  |0 1|2 3|4 5|7 6|  PLC outputs|
|-------------------------|----------------------------|---|---|---|---|-------------|
| maintance mode          | minimum detection          | 0 | 0 | 0 | x |   10101010  |
| normal operation        | normal detection           | 0 | 0 | 1 | x |   10011010  |
| front docking           | minimum area on front      | 0 | 1 | 0 | 0 |   10100110  |
| rear docking            | minimum area on rear       | 0 | 1 | 0 | 1 |   01100110  |
| front extension         | large area on front        | 0 | 1 | 1 | 0 |   01101001  |
| rear extension          | large area on rear         | 0 | 1 | 1 | 1 |   01010110  |
| right curve front       | large area on front right  | 1 | 0 | 0 | 0 |   10101001  |
| rigth curve rear        | large area on rear right   | 1 | 0 | 0 | 1 |   01101001  |
| left curve front        | large area on front left   | 1 | 0 | 1 | 0 |   10011001  |
| left curve rear         | large area on              | 1 | 0 | 1 | 1 |   01011001  |
| total extension         |                            | 1 | 1 | 1 | x |   10010101  |

obs: on LS front, wires normal couples (C, D, E and F). On LS rear, wire inverted the couple F (6 - 7 => 7 - 6), inverting the signal. 

### 5: Mission FEEDBACK status
```c
typedef enum {
     NO_MISSION = 0,
     MISSION_RUNNING = 1, 
     MISSION_ERROR = 2,
     MISSION_PAUSE = 3,
     MISSION_CANCEL = 4,
     MISSION_FINISH = 5
}    mission_status_tp;
```

### 6: Type of message
```c
typedef enum {
     MESSAGE_INFO = 0,
     MESSAGE_WARNING = 1,
     MESSAGE_ERROR = 2,
     MESSAGE_FATAL_ERROR = 3
}    message_tp;
```



### 7: Left Encoder and Right Encoder

32 bits signed integer (-2147483648 .. 2147483647) pulses<br>
64 int signed integer when use virtual counter (get_..._encoder_counts())
Encoder used: 10000 PPR (PULSES PER REVOLUTION) A + B, 
40000 CPR (COUNTS PER REVOLUTION), 
Wheel diameter: 250mm, 
Wheel perimeter: 785.4mm, 
785.4mm / 40000 = 0.019634954mm/count, 
or 50.929581789 pulses per mm.<br>

### 8: Timestamp

32 bits of a time of day, milliseconds of day, in raw data (data received from PLC)
64 bits in virtual (lib) timestamp, sincronized with raw data. 

### 9: joystick keys

Eather key (up, down, left, right) in 8 bits signed integer (-128 .. 127).<br>
Using digital inputs of PLC, when released, go to zero. <br>
When pressed, auto increment until max value. Sugested incr. in 100Hz<br> 


### 10: load/unload timeout 

0 - no timeout. Load until receive a stop command.<br>
1 .. 255 - timeout, in seconds. If undetect load or unload (by sensors) on timeout seconds, stop table motor and set an error code.<br>


--------------------------------------------------

# PLC (SIEMENS TIA PORTAL) PROGRAM. How to wire/integrate?


<span style="color:lightYellow">


# LOGIC FOR PLC


|modbus register                        |link to ...                                |
|---------------------------------------|-------------------------------------------|
|HOLD_REG.LEFT_ENC                      |left wheel encoder HSC
|HOLD_REG.RIGHT_ENC	                    |right wheel encoder HSC
|HOLD_REG.TIMESTAMP	                    |timestamp (miliseconds from start or from 0:00:00.000)
|HOLD_REG.JOY_UP	                    |joy up. 0 to released. 1 to 127 inc 0.01s while pressing.
|HOLD_REG.JOY_DOWN	                    |joy down. 0 to released. 1 to 127 inc 0.01s while pressing. 
|HOLD_REG.JOY_LEFT	                    |joy left. 0 to released. 1 to 127 inc 0.01s while pressing.
|HOLD_REG.JOY_RIGHT	                    |joy right. 0 to released. 1 to 127 inc 0.01s while pressing.
|HOLD_REG.DOC_STATUS.FRONT              |set when docked front detection by a low level sensor
|HOLD_REG.DOC_STATUS.REAR               |set when docked rear detection by a low level sensor
|HOLD_REG.DOC_STATUS.CHARGING           |set when in charge station
|HOLD_REG.DI.FRONT_BIT0                 |IO 0 from FRONT safety relay / LS
|HOLD_REG.DI.FRONT_BIT1                 |IO 1 from FRONT safety relay / LS
|HOLD_REG.DI.FRONT_BIT2                 |IO 2 from FRONT safety relay / LS
|HOLD_REG.DI.FRONT_BIT3                 |IO 3 from FRONT safety relay / LS
|HOLD_REG.DI.REAR_BIT0                  |IO 0 from REAR safety relay / LS
|HOLD_REG.DI.REAR_BIT1                  |IO 1 from REAR safety relay / LS
|HOLD_REG.DI.REAR_BIT2                  |IO 2 from REAR safety relay / LS
|HOLD_REG.DI.REAR_BIT3                  |IO 3 from REAR safety relay / LS
|HOLD_REG.DI.AUTO_SW                    |Digital input connected to auto switch
|HOLD_REG.DI.MAN_SW                     |Digital input connected to manual switch
|HOLD_REG.DI.RESET_SW                   |Digital input connected to reset button
|HOLD_REG.DI.TABLE_FRONT_LOAD           |Digital input connected to front table sensor
|HOLD_REG.DI.TABLE_REAR_LOAD            |Digital input connected to rear table sensor
|HOLD_REG.DI.JOYSTICK_CONNECTED         |Digital input connected to joystick detection (if exists)
|HOLD_REG.DI.LOADED                     |A system flag that register LOADED or NOT LOADED
|HOLD_REG.DI.CHARGING                   |Digital input from a charging detection system
|HOLD_REG.ERROR_CODE.RESET_WAIT         |Fail from safety detected and must press reset to continue
|HOLD_REG.ERROR_CODE.UNDEFINED          |An unmapped error
|HOLD_REG.ERROR_CODE.INITIAL_STATUS     |Until first reset press. Flag to initialization, reset detect.
|HOLD_REG.ERROR_CODE.LEFT_MOTOR         |Fail from left motor driver
|HOLD_REG.ERROR_CODE.RIGHT_MOTOR        |Fail from right motor driver
|HOLD_REG.ERROR_CODE.TABLE              |Fail from table driver
|HOLD_REG.ERROR_CODE.LOAD               |Timeout on load operation in top table
|HOLD_REG.ERROR_CODE.UNLOAD             |Timeout on unload operation in top table
|HOLD_REG.ERROR_CODE.EMERGENCY          |Digital input connected to Safety Relay
|HOLD_REG.ERROR_CODE.BATTERY            |Fail from battery (undervoltge, low current etc)
|HOLD_REG.ERROR_CODE.DOCKING            |Fail when docking detected by low level 
|HOLD_REG.ERROR_CODE.MAINTANCE          |Logic signaling of maintance mode. True to maintance mode.
|HOLD_REG.SEND_COMMAND                  |Send some commands to PC. Ex: send 64 to PC shutdown.
|HODL_REG.MISSION_TYPE                  |Selected by a HMI or other, set with type of mission (load, unload, move only, rear, back etc).
|                                       |Obs: Mission type bit 0 used to cancel the current mission.
|HOLD_REG.MISSION_SEQ                   |Increase this after change mission_type and mission_label to send a new mission command. Do not worry about byte overflow.
|HOLD_REG.MISSION_LABEL                 |Send 4 chars to identify a label (on map) to mission destination
| ------------------------------------  |
|HOLD_REG.FRONT_RED_LIGHTS              |To front red lights control
|HOLD_REG.REAR_RED_LIGHTS               |To rear red light control
|HOLD_REG.FRONT_GREEN_LIGHTS            |To front green lights control
|HOLD_REG.REAR_GREEN_LIGHTS             |To rear red light control
|HOLD_REG.LEFT_YELLOW_LIGHTS            |To yellow left lights control
|HOLD_REG.RIGHT_YELLOW_LIGHTS           |To yellow right lights control
|HOLD_REG.BUZZER                        |To buzzer
|HOLD_REG.LOAD_UNLOAD_TIMEOUT           |To LOAD/UNLOAD timer preset 
|HOLD_REG.COMMAND                       |To LOAD/UNLOAD control, error reset and to HOLD_REG.IOs.LOADED.
|                                       |obs: value 112 means PC is shutting down. 
|HOLD_REG.LS_SELECT                     |send signal to both laser scanners
|HOLD_REG.MISSION_FB_SET                |PC send to PLC a new value on a new feedback status
|HOLD_REG.MISSION_FB_STATUS             |Mission status. Here, can see some error, running, pause, finish etc.
|HOLD_REG.MESSAGE_TP                    |Type of text message. Use this to change color and signaling of received messages (info, warning, error etc)
|HOLD_REG.MESSAGE_COD                   |A single byte numeric code, to use with a errorlevel or error status code, showing this in HMI
|HOLD_REG.MESSAGE_TXT                   |A 20 chars string, with the message received from PC to display in HMI.

</span>

## VERSIONS / LOGS

18 oct 2024 - versão inicial (v0)   
28 oct 2024 - corrections in PLC Holding registers description   
18 oct 2024 - added bit HOLD_REG.IOs.LOADED and respective methods    
25 nov 2024 - modify LS status read register do get 4 bits from LS front and 4 bits from LS rear, in same byte   
26 nov 2024 - refactored code, new library version   
17 jan 2025 - modify encoder counter from int32 to int64, to prevent count overflow. Add set methods to atribute a value to enconder (int 64 bits)
22 jan 2025 - modify timestamp to uint64 (ms), based on data received from PLC (int32). Add set method to atribute a value to 64 bits timestamp.
30 jan 2025 - documentation fix
17 jul 2025 - inlcuded mission, message and commands data

