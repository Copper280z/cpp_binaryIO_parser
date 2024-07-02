// #include <stdio.h>
//
// #define PROJECT_NAME "serial_parser"
//
// int main(int argc, char **argv) {
//     if(argc != 1) {
//         printf("%s takes no arguments.\n", argv[0]);
//         return 1;
//     }
//     printf("This is project %s.\n", PROJECT_NAME);
//     return 0;
// }

#include <chrono>
#include <cstddef>
#include <cstdint>
// #include <iterator>
#include <vector>
#define TERMINAL    "/dev/ttyACM0"

#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
// #include <iostream>

#include "SimpleFOCRegisters.hpp"

/* define local struct for tty
 * load state into that struct
 * do stuff to it
 * set the state of the tty from the struct */
int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

/* define local struct for tty
 * load state into that struct
 * do stuff to it
 * set the state of the tty from the struct */
void set_mincount(int fd, int mcount)
{
    struct termios tty;

    /* Put the state of FD into *TERMIOS_P.  */
    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    // cc_t c_cc[NCCS];		/* control characters */
    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    /* Set the state of FD to *TERMIOS_P.
       Values for OPTIONAL_ACTIONS (TCSA*) are in <bits/termios.h>.  */
    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}

// sync len  type  id   [1   2   3   4]   [1    2    3    4]    [1    2    3    4]    [1    2    3    4]    [1    2    3    4]
// 0xa5 0x16 0x54 0x0   0x0 0x0 0x0 0x0   0xd9 0xa0 0xd5 0xbc   0x11 0x99 0x10 0xc0   0xb4 0x9a 0x81 0x3c   0x52 0xa1 0x11 0x3c
// 0xa5 0x16 0x54 0x0   0x0 0x0 0x0 0x0   0x6a 0xfd 0xba 0x3e   0x2b 0xe4 0xe4 0x41   0xb1 0x3c 0xbd 0xbe   0x31 0x77 0x62 0xbc
// 0xa5 0x16 0x54 0x0   0x0 0x0 0x0 0x0   0x80 0xdb 0xbb 0x3e   0x17 0x99 0xe1 0x41   0x40 0x47 0xb6 0xbe   0x5c 0xc4 0x5c 0xbb
#define MARKER_BYTE 0xA5
SimpleFOCRegisters regs = SimpleFOCRegisters();

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
    std::vector<Number> operands;
    Sample(int size) {
        operands.reserve(size);
    }

    void print_ops() {
        
        for (auto num : operands) {
            
            uint8_t reg_type = regs.typeOfRegister(num.reg);
        
            switch(reg_type)
            {
                case RegType::FLOAT:
                    printf("%.3f ", num.f);
                    break;
                case RegType::UINT32:
                    printf("%u ", num.u32);
                    break;
                case RegType::INT32:
                    printf("%i ", num.i32);
                    break;
                case RegType::UINT8:
                    printf("%u ", num.u8);
                    break;
            } // switch(reg_type)

        } // for num : operands
        printf("\n");
    } // print_ops
};

class TelemetryConfig {
public:
    uint8_t num_operands=0;
    std::vector<uint8_t> operand_registers;
    size_t total_bytes=0;

    void update_total() {
        size_t accumulator = 0;
        for (auto reg : operand_registers) {
            accumulator += regs.sizeOfRegister(reg);
        }
        total_bytes = accumulator;
    }
};

uint32_t search_for_marker(std::vector<uint8_t> buffer) {
    // skip the first byte in case we got here from a path that had a valid
    // sync byte but still needs to skip this frame and go to the next one
    for (size_t idx=1; idx<buffer.size(); idx++) {
        uint8_t byte = buffer[idx];
        // printf("0x%x ",byte);
        if (byte == MARKER_BYTE) {
            // printf(" found at: %lu\n", idx);
            return idx;
        }
    }
    // printf("\n");
    // printf("dump whole buffer\n");
    return buffer.size();
}

class ParseResult {
public:
    bool success = false;
    uint8_t status = 0;
    uint32_t bytes_used = 0;
};

/* return number of bytes consumed*/
ParseResult parse_frame(std::vector<uint8_t> buffer, Sample &sample, TelemetryConfig &conf) {
    // printf("Starting frame search with %lu bytes\n", buffer.size()); 
    ParseResult ret;
    static int counter = 0;

    size_t idx = 0;

    uint8_t byte = buffer[idx];
    idx+=1;

    if (byte != MARKER_BYTE) {
        // search for the next MARKER_BYTE
        // printf("bad start byte: 0x%x\n",byte);
        uint32_t bytes_used = search_for_marker(buffer);
        ret.success=false; 
        ret.status=1;
        ret.bytes_used=bytes_used;
        return ret;
    }

    uint8_t frame_length = buffer[idx];
    idx+=1;
    byte = buffer[idx];
    idx+=1;

    if (byte != FrameType::TELEMETRY) {
        // search for the next MARKER_BYTE
        // printf("0x%x - %c: Not a Telemetry Frame\n", byte, byte);
        // printf("0x%x 0x%x 0x%x\n", buffer[0], buffer[1], buffer[2]);
        // printf("%d frames since last\n", counter);
        counter = 0;
        uint32_t bytes_used = search_for_marker(buffer);
        ret.success=false; 
        ret.status=2;
        ret.bytes_used=bytes_used;
        return ret;
    }

    if ((conf.total_bytes+2) != frame_length) {
        printf("Frame length not equal to config: %lu - %u\n", conf.total_bytes+2, frame_length);
        uint32_t bytes_used = search_for_marker(buffer);

        ret.success=false; 
        ret.status=3;
        ret.bytes_used=bytes_used;
        return ret;
    }

    if ((frame_length+(size_t)2) > buffer.size()) {
        // printf("Frame longer than buffer\n");
        ret.success=false; 
        ret.status=4;
        ret.bytes_used=0;
        return ret;
    }


    uint8_t motor_addr = buffer[idx];
    idx+=1;
    (void) motor_addr;

    for (auto reg : conf.operand_registers) {
        auto reg_size = regs.sizeOfRegister(reg);
        uint8_t reg_type = regs.typeOfRegister(reg);
        sample.num+=1;
        Number val;
        val.reg = reg;

        val.type = reg_type;
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
    counter +=1;
    ret.success=true; 
    ret.status=0;
    ret.bytes_used=idx;
    return ret;
}

int main(int argc, char **argv)
{
    // TODO - take args maybe, if this stays an executable
    (void) argc;
    (void) argv;

    typedef std::chrono::high_resolution_clock Time;
    typedef std::chrono::milliseconds ms;
    auto t0 = Time::now();
    auto t1 = Time::now();

    // open and configure the serial port
    char portname[] = TERMINAL;
    int fd;

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }

    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    // baud doesn't matter because using USB right now
    set_interface_attribs(fd, B115200);

    // instance and manually set the telemetry configuration
    // TODO - ask the device for it's config and set this from that
    TelemetryConfig telem_conf;

    telem_conf.num_operands = 5;
    telem_conf.operand_registers.push_back(SimpleFOCRegister::REG_TARGET);
    telem_conf.operand_registers.push_back(SimpleFOCRegister::REG_ANGLE);
    telem_conf.operand_registers.push_back(SimpleFOCRegister::REG_VELOCITY);
    telem_conf.operand_registers.push_back(SimpleFOCRegister::REG_CURRENT_Q);
    telem_conf.operand_registers.push_back(SimpleFOCRegister::REG_CURRENT_D);
    telem_conf.update_total();

    // setup some stuff to use in the main loop
    std::vector<uint8_t> bytes_to_parse;
    float fps=0; 
    size_t frame_counter = 0;
    Sample sample(telem_conf.num_operands);
    ParseResult parse_result;

    do {
        uint8_t read_buf[512];
        int rdlen;

        rdlen = read(fd, read_buf, sizeof(read_buf) - 1);
        if (rdlen > 0) 
        { // we have some data to work with
            // put all the bytes from the read buffer into the end of the parse buffer
            bytes_to_parse.insert(bytes_to_parse.end(), read_buf, read_buf+rdlen);
            while (bytes_to_parse.size() > 0) {
                // clear out our previous sample
                sample.operands.clear();
                sample.num=0;

                // try to parse a frame
                parse_result = parse_frame(bytes_to_parse, sample, telem_conf);
                if (parse_result.bytes_used == 0) {
                    break; // need more data for a frame
                } 
                else {
                    // remove the bytes we used from the front of the parse buffer
                    // TODO - this is an inefficient use of an std::vector, 
                    //        maybe do something different for the parse buffer
                    bytes_to_parse.erase(bytes_to_parse.begin(),bytes_to_parse.begin()+parse_result.bytes_used);

                    // check the time and see if we should print anything
                    if (parse_result.success) {
                        t1 = Time::now();
                        auto et = std::chrono::duration_cast<ms>(t1-t0).count();
                        if ( et >= 500) {
                            fps = (float)frame_counter/((float)et/1000);
                            printf("got %.3f frames per sec\n", fps);
                            sample.print_ops();
                            t0=t1;
                            frame_counter = 0;
                        } // if (et >=500) ..... timer for prints
                        frame_counter+=1;
                    } // parse_result.success
                } // else  ..... if (bytes_used == 0)
            } // while (bytes_to_parse.size() > 0)
        } // if (rdlen > 0)
        else if (rdlen < 0) {
            printf("Error from read: %d: %s\n", rdlen, strerror(errno));
        } else {  /* rdlen == 0 */
            printf("Timeout from read\n");
        }

    } while (1);
}
