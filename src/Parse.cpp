#include "Parse.hpp"
#include "SimpleFOCRegisters.hpp"
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
        // maybe check to make sure the frame length is below some upper bound
        // to protect against absurd message corruptions?
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
            // the host doesn't ever receive register frames from the device?
            sample.address=address;
            ret.frame_type = FrameType::REGISTER;
            bytes_used = parse_register_frame(buffer, sample, 4);
            break;
        case FrameType::HEADER:
            // seems to only be sent when device inits, or when host changes
            // headers on the device, or when the host requests the register
            // REG_TELEMETRY_REG, in which case the device sends both a response
            // frame and a header frame.
            // TODO - set a status flag when host requests a header change and 
            //          only clear it once we get a valid header response from 
            //          the device. All telemetry frames should be assumed 
            //          invalid when the flag is active.
            ret.frame_type = FrameType::HEADER;
            bytes_used = parse_header_frame(buffer, 4);
            break;
        case FrameType::RESPONSE:
            // received from device in response to a register frame
            // seems to be the same format as a register frame
            sample.address=address;
            ret.frame_type = FrameType::RESPONSE;
            bytes_used = parse_response_frame(buffer, sample, 4);
            break;
        case FrameType::SYNC:
            // received from device in response to a SYNC frame
            //      - not really sure what to do with this?
        case FrameType::ALERT:
            // seems to not be used?
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
    // right now the device doesn't return any doubles, but that's probably not a safe long term assumption
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
    } else {    // reg_size
        if (reg_size == 255) {      // hacky way to define a telemetry reg response
            // sync  len  type  reg  num_regs  [motor_id, reg_id]... 
            // size from packet is 2*num_regs+1+1
            // reg format is motor_idx, register_id

            // not sure what to do with this for now, so lets just print it
            // actionable data comes from the header frame that follows
            uint32_t msg_len = buffer[1]+2;
            size_t usable_len = 0;
            if (msg_len < buffer.size()) {
                usable_len = msg_len;
            } else {
                usable_len = buffer.size();
            }
            
            printf("Got register frame with header info: \n");
            for (size_t i=0;i<usable_len;i++) {
                printf("0x%x ", buffer[i]);
            }
            printf("\n");
        }

    }
    idx+=reg_size;
    sample.operands.push_back(val);
    return idx;
}

uint8_t BinaryIOParser::parse_response_frame(std::vector<uint8_t> const buffer, Sample &sample, uint8_t start_idx) {
    printf("got a response frame!\n");
    return parse_register_frame(buffer, sample, start_idx);
}

uint8_t BinaryIOParser::parse_header_frame(std::vector<uint8_t> const buffer, uint8_t start_idx) {
    printf("got a header frame!\n");
    telem_conf.operand_motors.clear();
    telem_conf.operand_registers.clear();
    telem_conf.num_operands=0;

    uint32_t msg_len = buffer[1];
    if (msg_len > buffer.size()) {
        return 0;
    }

    uint8_t idx = start_idx;
    for (size_t i=idx;i<msg_len+2; i+=2) {
        if (i+1 < buffer.size()) {
            telem_conf.operand_motors.push_back(buffer[i]);
            telem_conf.operand_registers.push_back(buffer[i+1]); 
            telem_conf.num_operands+=1; 
            idx+=2;
        } else {
            printf("Something went wrong with reading a header frame, tried to index OOB\n");
            uint32_t msg_len = buffer[1]+2;
            size_t usable_len = 0;
            if (msg_len < buffer.size()) {
                usable_len = msg_len;
            } else {
                usable_len = buffer.size();
            }
            
            printf("Here's the whole message buffer: \n");
            for (size_t i=0;i<usable_len;i++) {
                printf("%x ", buffer[i]);
            }
            printf("\n");
        }
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

std::vector<uint8_t> BinaryIOParser::encode_frame(Sample sample) {
    std::vector<uint8_t> ret;
    ret.push_back(MARKER_BYTE);
    if (sample.address == SimpleFOCRegister::REG_TELEMETRY_REG) {
        // each Number instance should be a u8 representing a motor idx or register
        ret.push_back((uint8_t) (2+sample.operands.size())); 
        ret.push_back(FrameType::REGISTER);
        ret.push_back(SimpleFOCRegister::REG_TELEMETRY_REG);
        for (auto val : sample.operands) {
            // invalid frame
            if (val.type != RegType::UINT8) {
                ret.clear();
                return ret;
            }
            ret.push_back(val.u8);
        }
    } else { // if sample.address....
             
        if (sample.operands.size() != 1) {
            ret.clear();
            return ret;
        }
        Number val = sample.operands[0];
        uint8_t val_size = regs->sizeOfRegister(val.reg);
        ret.push_back((uint8_t) (val_size+1)); 
        ret.push_back(FrameType::REGISTER);
        ret.push_back(val.reg);
        uint8_t * p;
        uint8_t n_bytes = 0; 
        switch(val.type)
        {
            case RegType::FLOAT:
                p = reinterpret_cast<uint8_t*>(&val.f);
                n_bytes=4;
                break;
            case RegType::UINT32:
                p = reinterpret_cast<uint8_t*>(&val.u32);
                n_bytes=4;
                break;
            case RegType::INT32:
                p = reinterpret_cast<uint8_t*>(&val.i32);
                n_bytes=4;
                break;
            case RegType::UINT8:
                p = reinterpret_cast<uint8_t*>(&val.u8);
                n_bytes=1;
                break;
            default:
                ret.clear();
                return ret;
        }
        for (size_t i=0;i<n_bytes;i++) {
            ret.push_back(p[i]);
        }

    }

    return ret;
}
