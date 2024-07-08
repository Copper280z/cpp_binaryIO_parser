#pragma once

#include <stdlib.h>
#include <vector>

#include "SimpleFOCRegisters.hpp"

#define MARKER_BYTE 0xA5

struct Number {
    uint8_t reg;
    uint8_t type;
    union {
        float f;
        uint32_t u32;
        int32_t i32;
        uint8_t u8;
    };
};

class Sample {
    public:
        uint8_t num=0;
        uint8_t frame_type = 0;
        uint8_t address=0;
        std::vector<Number> operands;
        Sample(int size);
        void clear();
};

class TelemetryConfig {
    public:
        TelemetryConfig(SimpleFOCRegisters * const _regs);

        uint8_t num_operands=0;
        std::vector<uint8_t> operand_motors;
        std::vector<uint8_t> operand_registers;
        size_t total_bytes=0;

        void update_total();
    private:
        SimpleFOCRegisters * const regs;
};

class ParseResult {
    public:
        enum {SUCCESS, BAD_START_BYTE, CONF_MISMATCH, NOT_ENOUGH_BYTES, UNHANDLED_FRAME};
        bool success = false;
        uint8_t status = 0;
        uint8_t frame_type = 0;
        uint32_t bytes_used = 0;
};
class BinaryIOParser {
    public:
        SimpleFOCRegisters * const regs;
        TelemetryConfig &telem_conf;
        BinaryIOParser(SimpleFOCRegisters * const _regs, TelemetryConfig &_telem_conf);
        ParseResult parse_frame(std::vector<uint8_t> buffer, Sample &sample);
        std::vector<uint8_t> encode_frame(Sample sample);

    private:
        uint8_t parse_telemetry_frame(std::vector<uint8_t> const buffer, Sample &sample, uint8_t start_idx);
        uint8_t parse_register_frame(std::vector<uint8_t> const buffer, Sample &sample, uint8_t start_idx);
        uint8_t parse_response_frame(std::vector<uint8_t> const buffer, Sample &sample, uint8_t start_idx);
        uint8_t parse_header_frame(std::vector<uint8_t> const buffer, uint8_t start_idx);
        uint8_t parse_sync_frame(std::vector<uint8_t> const buffer, Sample &sample, uint8_t start_idx);
        uint8_t parse_alert_frame(std::vector<uint8_t> const buffer, Sample &sample, uint8_t start_idx);

};
