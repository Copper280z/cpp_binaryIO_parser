#include "Parse.hpp"
#include <cstdint>


Sample::Sample(int size) {
    operands.reserve(size);
}

void Sample::clear() {
    operands.clear();
    num=0;
}

TelemetryConfig::TelemetryConfig(SimpleFOCRegisters * const _regs) : regs(_regs) {};

void TelemetryConfig::update_total() {
    size_t accumulator = 0;
    for (auto reg : operand_registers) {
        accumulator += regs->sizeOfRegister(reg);
    }
    total_bytes = accumulator;
}

static uint32_t search_for_marker(std::vector<uint8_t> buffer) {
    // skip the first byte in case we got here from a path that had a valid
    // sync byte but still needs to skip this frame and go to the next one
    for (size_t idx=1; idx<buffer.size(); idx++) {
        uint8_t byte = buffer[idx];
        if (byte == MARKER_BYTE) {
            return idx;
        }
    }
    return buffer.size();
}

BinaryIOParser::BinaryIOParser(SimpleFOCRegisters * const _regs, TelemetryConfig &_telem_conf) :
    regs(_regs), telem_conf(_telem_conf) {};

ParseResult BinaryIOParser::parse_frame(std::vector<uint8_t> buffer, Sample &sample) {
    ParseResult ret;

    uint8_t byte = buffer[0];

    if (byte != MARKER_BYTE) {
        // search for the next MARKER_BYTE
        // printf("bad start byte: 0x%x\n",byte);
        uint32_t bytes_used = search_for_marker(buffer);
        ret.success=false; 
        ret.status=ParseResult::BAD_START_BYTE;
        ret.bytes_used=bytes_used;
        return ret;
    }

    uint8_t frame_length = buffer[1];

    if ((telem_conf.total_bytes+2) != frame_length) {
        printf("Frame length not equal to config: %lu - %u\n", telem_conf.total_bytes+2, frame_length);
        uint32_t bytes_used = search_for_marker(buffer);

        ret.success=false; 
        ret.status=ParseResult::CONF_MISMATCH;
        ret.bytes_used=bytes_used;
        return ret;
    }

    if ((frame_length + (size_t) 2) > buffer.size()) {
        // printf("Frame longer than buffer\n");
        ret.success=false; 
        ret.status=ParseResult::NOT_ENOUGH_BYTES;
        ret.bytes_used=0;
        return ret;
    }

    uint8_t frame_type = buffer[2];
    uint8_t motor_addr = buffer[3];
    (void) motor_addr;
    

    uint32_t bytes_used = 0;
    switch (frame_type) {
        case FrameType::TELEMETRY:
            bytes_used = parse_telemetry_frame(buffer, sample, 4);
            break;
        case FrameType::REGISTER:
        case FrameType::RESPONSE:
        case FrameType::HEADER:
        case FrameType::SYNC:
        case FrameType::ALERT:
        default:
            bytes_used = search_for_marker(buffer);
            ret.success=false; 
            ret.status=ParseResult::UNHANDLED_FRAME;
            ret.bytes_used=bytes_used;
            return ret;

    }

    ret.success=true; 
    ret.status=ParseResult::SUCCESS;
    ret.bytes_used=bytes_used;
    return ret;
}

uint8_t parse_register_frame(std::vector<uint8_t> const buffer, Sample &sample, uint8_t start_idx);
uint8_t parse_response_frame(std::vector<uint8_t> const buffer, Sample &sample, uint8_t start_idx);
uint8_t parse_header_frame(std::vector<uint8_t> const buffer, Sample &sample, uint8_t start_idx);
uint8_t parse_sync_frame(std::vector<uint8_t> const buffer, Sample &sample, uint8_t start_idx);
uint8_t parse_alert_frame(std::vector<uint8_t> const buffer, Sample &sample, uint8_t start_idx);

uint8_t BinaryIOParser::parse_telemetry_frame(std::vector<uint8_t> const buffer, Sample &sample, uint8_t start_idx) {

    uint8_t idx = start_idx;
    for (auto reg : telem_conf.operand_registers) {
        auto reg_size = regs->sizeOfRegister(reg);
        uint8_t reg_type = regs->typeOfRegister(reg);
        sample.num+=1;
        Number val;
        val.reg = reg;

        val.type = reg_type;
        // array telemetry regs are not currently handled
        if (reg_size <= 4) { 
            switch(reg_type)
            {
                case RegType::FLOAT:
                    memcpy(&val.f, &buffer[idx], reg_size);
                    break;
                case RegType::UINT32:
                    memcpy(&val.u32, &buffer[idx], reg_size);
                    break;
                case RegType::INT32:
                    memcpy(&val.i32, &buffer[idx], reg_size);
                    break;
                case RegType::UINT8:
                    memcpy(&val.u8, &buffer[idx], reg_size);
                    break;
            }
        }
        idx+=reg_size;
        sample.operands.push_back(val);
    }
    return idx;
};
