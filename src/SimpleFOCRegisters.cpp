
#include "./SimpleFOCRegisters.hpp"
// #include "BLDCMotor.h"
// #include "./telemetry/Telemetry.h"


SimpleFOCRegisters::SimpleFOCRegisters(){};
SimpleFOCRegisters::~SimpleFOCRegisters(){};

// return the size of the register values, when output to comms, in bytes
uint8_t SimpleFOCRegisters::sizeOfRegister(uint8_t reg){
    switch (reg) {
        case SimpleFOCRegister::REG_TARGET:
        case SimpleFOCRegister::REG_ANGLE:
        case SimpleFOCRegister::REG_VELOCITY:
        case SimpleFOCRegister::REG_SENSOR_ANGLE:
        case SimpleFOCRegister::REG_SENSOR_MECHANICAL_ANGLE:
        case SimpleFOCRegister::REG_SENSOR_VELOCITY:
        case SimpleFOCRegister::REG_SENSOR_TIMESTAMP:        
        case SimpleFOCRegister::REG_VOLTAGE_Q:
        case SimpleFOCRegister::REG_VOLTAGE_D:
        case SimpleFOCRegister::REG_CURRENT_Q:
        case SimpleFOCRegister::REG_CURRENT_D:
        case SimpleFOCRegister::REG_CURRENT_A:
        case SimpleFOCRegister::REG_CURRENT_B:
        case SimpleFOCRegister::REG_CURRENT_C:
        case SimpleFOCRegister::REG_VEL_PID_P:
        case SimpleFOCRegister::REG_VEL_PID_I:
        case SimpleFOCRegister::REG_VEL_PID_D:
        case SimpleFOCRegister::REG_VEL_PID_LIM:
        case SimpleFOCRegister::REG_VEL_PID_RAMP:
        case SimpleFOCRegister::REG_VEL_LPF_T:
        case SimpleFOCRegister::REG_ANG_PID_P:
        case SimpleFOCRegister::REG_ANG_PID_I:
        case SimpleFOCRegister::REG_ANG_PID_D:
        case SimpleFOCRegister::REG_ANG_PID_LIM:
        case SimpleFOCRegister::REG_ANG_PID_RAMP:
        case SimpleFOCRegister::REG_ANG_LPF_T:
        case SimpleFOCRegister::REG_CURQ_PID_P:
        case SimpleFOCRegister::REG_CURQ_PID_I:
        case SimpleFOCRegister::REG_CURQ_PID_D:
        case SimpleFOCRegister::REG_CURQ_PID_LIM:
        case SimpleFOCRegister::REG_CURQ_PID_RAMP:
        case SimpleFOCRegister::REG_CURQ_LPF_T:
        case SimpleFOCRegister::REG_CURD_PID_P:
        case SimpleFOCRegister::REG_CURD_PID_I:
        case SimpleFOCRegister::REG_CURD_PID_D:
        case SimpleFOCRegister::REG_CURD_PID_LIM:
        case SimpleFOCRegister::REG_CURD_PID_RAMP:
        case SimpleFOCRegister::REG_CURD_LPF_T:
        case SimpleFOCRegister::REG_VOLTAGE_LIMIT:
        case SimpleFOCRegister::REG_CURRENT_LIMIT:
        case SimpleFOCRegister::REG_VELOCITY_LIMIT:
        case SimpleFOCRegister::REG_DRIVER_VOLTAGE_LIMIT:
        case SimpleFOCRegister::REG_DRIVER_VOLTAGE_PSU:
        case SimpleFOCRegister::REG_VOLTAGE_SENSOR_ALIGN:
        case SimpleFOCRegister::REG_PWM_FREQUENCY:
        case SimpleFOCRegister::REG_ZERO_ELECTRIC_ANGLE:
        case SimpleFOCRegister::REG_ZERO_OFFSET:
        case SimpleFOCRegister::REG_PHASE_RESISTANCE:
        case SimpleFOCRegister::REG_KV:
        case SimpleFOCRegister::REG_INDUCTANCE:
        case SimpleFOCRegister::REG_TELEMETRY_DOWNSAMPLE:
        case SimpleFOCRegister::REG_ITERATIONS_SEC:
        case SimpleFOCRegister::REG_CURA_GAIN:
        case SimpleFOCRegister::REG_CURB_GAIN:
        case SimpleFOCRegister::REG_CURC_GAIN:
        case SimpleFOCRegister::REG_CURA_OFFSET:
        case SimpleFOCRegister::REG_CURB_OFFSET:
        case SimpleFOCRegister::REG_CURC_OFFSET:     
            return 4;
        case SimpleFOCRegister::REG_SYS_TIME:
            return 4; // TODO how big is millis()? Same on all platforms?
        case SimpleFOCRegister::REG_MOTION_DOWNSAMPLE:
        case SimpleFOCRegister::REG_SENSOR_DIRECTION:
        case SimpleFOCRegister::REG_POLE_PAIRS:
        case SimpleFOCRegister::REG_STATUS:
        case SimpleFOCRegister::REG_ENABLE:
        case SimpleFOCRegister::REG_MODULATION_MODE:
        case SimpleFOCRegister::REG_TORQUE_MODE:
        case SimpleFOCRegister::REG_CONTROL_MODE:
        case SimpleFOCRegister::REG_NUM_MOTORS:   
        case SimpleFOCRegister::REG_MOTOR_ADDRESS:
        case SimpleFOCRegister::REG_TELEMETRY_CTRL:
            return 1;
        case SimpleFOCRegister::REG_POSITION:
            return 8;
        case SimpleFOCRegister::REG_CURRENT_ABC:
        case SimpleFOCRegister::REG_PHASE_VOLTAGE:
            return 12;
        case SimpleFOCRegister::REG_PHASE_STATE:
            return 3;
        case SimpleFOCRegister::REG_TELEMETRY_REG:
            return 255; // the number of registers is sent first, then the address of each reg is sent
        case SimpleFOCRegister::REG_DRIVER_ENABLE:
        case SimpleFOCRegister::REG_ENABLE_ALL: // write-only
        default: // unknown register or write only register (no output) or can't handle in superclass
            return 0;
    }
};

uint8_t SimpleFOCRegisters::typeOfRegister(uint8_t reg){
    switch (reg) {
        case SimpleFOCRegister::REG_TARGET:
        case SimpleFOCRegister::REG_ANGLE:
        case SimpleFOCRegister::REG_VELOCITY:
        case SimpleFOCRegister::REG_SENSOR_ANGLE:
        case SimpleFOCRegister::REG_SENSOR_MECHANICAL_ANGLE:
        case SimpleFOCRegister::REG_SENSOR_VELOCITY:
        case SimpleFOCRegister::REG_SENSOR_TIMESTAMP:        
        case SimpleFOCRegister::REG_VOLTAGE_Q:
        case SimpleFOCRegister::REG_VOLTAGE_D:
        case SimpleFOCRegister::REG_CURRENT_Q:
        case SimpleFOCRegister::REG_CURRENT_D:
        case SimpleFOCRegister::REG_CURRENT_A:
        case SimpleFOCRegister::REG_CURRENT_B:
        case SimpleFOCRegister::REG_CURRENT_C:
        case SimpleFOCRegister::REG_VEL_PID_P:
        case SimpleFOCRegister::REG_VEL_PID_I:
        case SimpleFOCRegister::REG_VEL_PID_D:
        case SimpleFOCRegister::REG_VEL_PID_LIM:
        case SimpleFOCRegister::REG_VEL_PID_RAMP:
        case SimpleFOCRegister::REG_VEL_LPF_T:
        case SimpleFOCRegister::REG_ANG_PID_P:
        case SimpleFOCRegister::REG_ANG_PID_I:
        case SimpleFOCRegister::REG_ANG_PID_D:
        case SimpleFOCRegister::REG_ANG_PID_LIM:
        case SimpleFOCRegister::REG_ANG_PID_RAMP:
        case SimpleFOCRegister::REG_ANG_LPF_T:
        case SimpleFOCRegister::REG_CURQ_PID_P:
        case SimpleFOCRegister::REG_CURQ_PID_I:
        case SimpleFOCRegister::REG_CURQ_PID_D:
        case SimpleFOCRegister::REG_CURQ_PID_LIM:
        case SimpleFOCRegister::REG_CURQ_PID_RAMP:
        case SimpleFOCRegister::REG_CURQ_LPF_T:
        case SimpleFOCRegister::REG_CURD_PID_P:
        case SimpleFOCRegister::REG_CURD_PID_I:
        case SimpleFOCRegister::REG_CURD_PID_D:
        case SimpleFOCRegister::REG_CURD_PID_LIM:
        case SimpleFOCRegister::REG_CURD_PID_RAMP:
        case SimpleFOCRegister::REG_CURD_LPF_T:
        case SimpleFOCRegister::REG_VOLTAGE_LIMIT:
        case SimpleFOCRegister::REG_CURRENT_LIMIT:
        case SimpleFOCRegister::REG_VELOCITY_LIMIT:
        case SimpleFOCRegister::REG_DRIVER_VOLTAGE_LIMIT:
        case SimpleFOCRegister::REG_DRIVER_VOLTAGE_PSU:
        case SimpleFOCRegister::REG_VOLTAGE_SENSOR_ALIGN:
        case SimpleFOCRegister::REG_PWM_FREQUENCY:
        case SimpleFOCRegister::REG_ZERO_ELECTRIC_ANGLE:
        case SimpleFOCRegister::REG_ZERO_OFFSET:
        case SimpleFOCRegister::REG_PHASE_RESISTANCE:
        case SimpleFOCRegister::REG_KV:
        case SimpleFOCRegister::REG_INDUCTANCE:
        case SimpleFOCRegister::REG_TELEMETRY_DOWNSAMPLE:
        case SimpleFOCRegister::REG_ITERATIONS_SEC:
        case SimpleFOCRegister::REG_CURA_GAIN:
        case SimpleFOCRegister::REG_CURB_GAIN:
        case SimpleFOCRegister::REG_CURC_GAIN:
        case SimpleFOCRegister::REG_CURA_OFFSET:
        case SimpleFOCRegister::REG_CURB_OFFSET:
        case SimpleFOCRegister::REG_CURC_OFFSET:     
            return RegType::FLOAT;
        case SimpleFOCRegister::REG_SYS_TIME:
            return RegType::UINT32; // TODO how big is millis()? Same on all platforms?
        case SimpleFOCRegister::REG_MOTION_DOWNSAMPLE:
        case SimpleFOCRegister::REG_SENSOR_DIRECTION:
        case SimpleFOCRegister::REG_POLE_PAIRS:
        case SimpleFOCRegister::REG_STATUS:
        case SimpleFOCRegister::REG_ENABLE:
        case SimpleFOCRegister::REG_MODULATION_MODE:
        case SimpleFOCRegister::REG_TORQUE_MODE:
        case SimpleFOCRegister::REG_CONTROL_MODE:
        case SimpleFOCRegister::REG_NUM_MOTORS:   
        case SimpleFOCRegister::REG_MOTOR_ADDRESS:
        case SimpleFOCRegister::REG_TELEMETRY_CTRL:
            return RegType::UINT8;
        case SimpleFOCRegister::REG_POSITION:
            return RegType::FLOAT;
        case SimpleFOCRegister::REG_CURRENT_ABC:
        case SimpleFOCRegister::REG_PHASE_VOLTAGE:
            return RegType::FLOAT;
        case SimpleFOCRegister::REG_PHASE_STATE:
            return RegType::FLOAT;
        case SimpleFOCRegister::REG_TELEMETRY_REG:
            return RegType::UINT32; // the number of registers is sent first, then the address of each reg is sent
        case SimpleFOCRegister::REG_DRIVER_ENABLE:
        case SimpleFOCRegister::REG_ENABLE_ALL: // write-only
        default: // unknown register or write only register (no output) or can't handle in superclass
            return 0;
    }
};

SimpleFOCRegisters* SimpleFOCRegisters::regs = new SimpleFOCRegisters();

