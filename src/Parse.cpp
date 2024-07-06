#include "Parse.hpp"
#include <cstdint>
#include <cstring>
#include <stdio.h>

using namespace std;

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
    // -the first byte is never an acceptable place for the next marker to be
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

ParseResult BinaryIOParser::parse_frame(const std::vector<uint8_t> buffer, Sample &sample) {
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

    if ((frame_length + (size_t) 2) > buffer.size()) {
        // printf("Frame longer than buffer\n");
        ret.success=false; 
        ret.status=ParseResult::NOT_ENOUGH_BYTES;
        ret.bytes_used=0;
        return ret;
    }

    uint8_t frame_type = buffer[2];
    uint8_t address = buffer[3];

    uint32_t bytes_used = 0;
    switch (frame_type) {
        case FrameType::TELEMETRY:
            if ((telem_conf.total_bytes+2) != frame_length) {
                printf("Frame length not equal to config: %lu - %u\n", telem_conf.total_bytes+2, frame_length);
                uint32_t bytes_used = search_for_marker(buffer);

                ret.success=false; 
                ret.status=ParseResult::CONF_MISMATCH;
                ret.frame_type = FrameType::TELEMETRY;
                ret.bytes_used=bytes_used;
                return ret;
            }
            bytes_used = parse_telemetry_frame(buffer, sample, 4);
            sample.address=address;
            break;
        case FrameType::REGISTER:
            bytes_used = parse_register_frame(buffer, sample, 4);
            ret.frame_type = FrameType::REGISTER;
            sample.address=address;
            break;
        case FrameType::HEADER:
            ret.frame_type = FrameType::HEADER;
            bytes_used = parse_header_frame(buffer, 4);
            break;
        case FrameType::RESPONSE:
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

uint8_t BinaryIOParser::parse_register_frame(std::vector<uint8_t> const buffer, Sample &sample, uint8_t start_idx) {

    uint8_t reg = buffer[3];
    uint8_t idx = start_idx;
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
    return idx;
}
// uint8_t BinaryIOParser::parse_response_frame(std::vector<uint8_t> const buffer, Sample &sample, uint8_t start_idx);
uint8_t BinaryIOParser::parse_header_frame(std::vector<uint8_t> const buffer, uint8_t start_idx) {
    telem_conf.operand_motors.clear();
    telem_conf.operand_registers.clear();
    telem_conf.num_operands=0;
    uint8_t idx = start_idx;
    for (size_t i=start_idx;i<buffer.size()-start_idx; i+=2) {
        telem_conf.operand_motors.push_back(buffer[i]);
        telem_conf.operand_registers.push_back(buffer[i+1]); // should guard against OOB read explicitly
        telem_conf.num_operands+=1; 
        idx+=2;
    }
    return idx;
}
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
