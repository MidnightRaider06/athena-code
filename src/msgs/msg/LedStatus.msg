uint8 CMD_SOLID = 1
uint8 CMD_FLASH = 2
uint8 CMD_PULSE = 3
uint8 CMD_OFF   = 255

uint8 cmd
uint8 r
uint8 g
uint8 b
uint8 param   # rate in tenths-of-Hz for FLASH/PULSE; 0 = default (2 Hz)
