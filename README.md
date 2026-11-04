# CAN Control
Responds to UUID: CAN_UUID_HAND (0x60). 
Should respond to broadcasts addressed to motor and peripheral domains (set UUID = 0x00 and domain bits high for broadcast).

## Lasers
Both lasers can be set using `CANPeripheralPacket_SetPWMDutyCycle`, `peripheralId` = 0x01, `dutyCycle` between 0 (off) and 100 (full brightness). Values persist until changed again. Lasers are off at startup.

## Linear Actuator
The linear actuator can be driven using `CANPeripheralPacket_SetLinearActuator` with `peripheralId` = 0x02.
To drive forward, set `drive` > 0.
To drive backwards, set `drive` < 0.
To brake actuator, set `drive` = 0.
Values will persist for `ACTUATOR_TIMEOUT_MS` = 500 , after which actuator will brake. Actuators initial state is off. [^1]

## Hand Stepper 
The stepper motor for the hand can be driven using `CANMotorPacket_Stepper_DriveRevolutions`. The revolutions part is not currently implemented.
For `numRevolutions` > 0, hand motor will open.
For `numRevolutions` < 0, hand motor will close.
`numRevolutions` =0 0 disables the motor.
Values will persist for `MOTOR_TIMEOUT_MS ` = 500. Must send CAN packets more frequently than 500 ms for continuous driving. Motor drives at a default speed that is not configurable over CAN (yet).

### Limit Switch
Currently there is only an inner limit switch. It will trigger when the hand is fully closed, sending a `CANMotorPacket_LimitSwitchAlert` to CAN_UUID_JETSON once initially when pressed, with `motorId` = 0x01 and `switchStatus` = true.
Motor cannot be driven further inwards when limit switch is pressed. Requests to do so will result in additional `CANMotorPacket_LimitSwitchAlert` packets.


## Other Packets
- Sends a `CANUniversalPacket_HeartBeat` packet to CAN_UUID_JETSON every `HEARTBEAT_INTERVAL_MS` =  500. `error` is not defined yet and is always set to 0, `state` will be 1 if motor is running, 0 otherwise.
- `CANUniversalPacket_EStop`: Turns off Laser, Stepper, and sets Actuator to brake.
- `CANUniversalPacket_FirmwareVersion`: Responds with version, current revision should be 'hand4'.
- Board should acknowledge all packets sent with CAN_ACK set high with `CANUniversalPacket_Acknowledge`. `failure` will be high if command could not succesfully be actuated (either not recognized or not possible_), and low if sucessfully actuated.



### Other Notes
- For correct actuator directionality, IN2 should be conneted to USART_TX (outer pin on the board) and IN1 should be connected to USART_RX (pin to the left of TX). 
- Limit switch is active high, i.e. it is high impedance when pressed resulting in 3v3 to the board from pullup.
- Firmware is currently using the internal clock, which has lower accuracy, we should update and test with the external HSE.
- CAN led flashes for 20 ms when board receives CAN messages, DBG_LED should be one when motor is driving. 


[^1]: "Off" and "brake" are two different states for the actuator. My impression is that brake will hold the actuators given position even under some force, so after the timeout time, the actuator position will remain in its last setpoint. I think off means that the actuator will retreat to its default position. This is not currently set-able by CAN but may be added if needed.
