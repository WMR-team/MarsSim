Header header
uint8 DRIVE_MODE=0
uint8 IDENTIFY_MODE=1
uint8 mode
float64 speed           # DRIVE_MODE: 线速度 m/s
float64 omega           # DRIVE_MODE: 角速度 rad/s
float64[6] wheel_speeds # IDENTIFY_MODE: 6轮独立转速 rad/s (FL,FR,ML,MR,BL,BR)
