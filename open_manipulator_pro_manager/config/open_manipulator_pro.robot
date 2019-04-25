[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE
/dev/ttyUSB0 | 1000000  | joint1

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL           | PROTOCOL | DEV NAME | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 1   | H54P-200-S500-R | 2.0      | joint1   | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 2   | H54P-200-S500-R | 2.0      | joint2   | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 3   | H54P-100-S500-R | 2.0      | joint3   | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 4   | H54P-100-S500-R | 2.0      | joint4   | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 5   | H42P-020-S300-R | 2.0      | joint5   | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 6   | H42P-020-S300-R | 2.0      | joint6   | present_position, present_voltage
