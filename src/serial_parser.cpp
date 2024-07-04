#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>

#include "SimpleFOCRegisters.hpp"
#include "Parse.hpp"

#define TERMINAL    "/dev/cu.usbmodem2086308E484E1"

using namespace std;

class InputParser{
    public:
        InputParser (int &argc, char **argv){
            for (int i=1; i < argc; ++i)
                this->tokens.push_back(std::string(argv[i]));
        }
        /// @author iain
        const std::string& getCmdOption(const std::string &option) const{
            std::vector<std::string>::const_iterator itr;
            itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
            if (itr != this->tokens.end() && ++itr != this->tokens.end()){
                return *itr;
            }
            static const std::string empty_string("");
            return empty_string;
        }
        /// @author iain
        bool cmdOptionExists(const std::string &option) const{
            return std::find(this->tokens.begin(), this->tokens.end(), option)
                   != this->tokens.end();
        }
    private:
        std::vector <std::string> tokens;
};

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

// sync len  type  id   [1   2   3   4]   [1    2    3    4]    [1    2    3    4]    [1    2    3    4]    [1    2    3    4]
// 0xa5 0x16 0x54 0x0   0x0 0x0 0x0 0x0   0xd9 0xa0 0xd5 0xbc   0x11 0x99 0x10 0xc0   0xb4 0x9a 0x81 0x3c   0x52 0xa1 0x11 0x3c
// 0xa5 0x16 0x54 0x0   0x0 0x0 0x0 0x0   0x6a 0xfd 0xba 0x3e   0x2b 0xe4 0xe4 0x41   0xb1 0x3c 0xbd 0xbe   0x31 0x77 0x62 0xbc
// 0xa5 0x16 0x54 0x0   0x0 0x0 0x0 0x0   0x80 0xdb 0xbb 0x3e   0x17 0x99 0xe1 0x41   0x40 0x47 0xb6 0xbe   0x5c 0xc4 0x5c 0xbb
SimpleFOCRegisters regs = SimpleFOCRegisters();


void print_ops(Sample * const sample) {
    
    for (auto num : sample->operands) {
        
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




int main(int argc, char **argv)
{
    std::string portname(TERMINAL);

    // TODO - take args maybe, if this stays an executable
    InputParser input(argc, argv);
    if(input.cmdOptionExists("-h")){
        printf("Usage: \n");
        printf("-p SERIAL_PORT - ie /dev/ttyACM0 or /dev/cu.usbmodem[...]\n");
    }
    const std::string &port = input.getCmdOption("-p");
    if (!port.empty()){
        portname = port.c_str();
    }

    typedef std::chrono::high_resolution_clock Time;
    typedef std::chrono::milliseconds ms;
    auto t0 = Time::now();
    auto t1 = Time::now();

    // open and configure the serial port
    int fd;

    fd = open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname.c_str(), strerror(errno));
        return -1;
    }

    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    // baud doesn't matter because using USB right now
    set_interface_attribs(fd, B115200);

    // instance and manually set the telemetry configuration
    // TODO - ask the device for it's config and set this from that
    TelemetryConfig telem_conf(&regs);

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
    
    BinaryIOParser parser(&regs, telem_conf);

    do {
        uint8_t read_buf[512];
        int rdlen;

        rdlen = read(fd, read_buf, sizeof(read_buf) - 1);
        if (rdlen > 0) 
        { // we have some data to work with
            // put all the bytes from the read buffer into the end of the parse buffer
            bytes_to_parse.insert(bytes_to_parse.end(), read_buf, read_buf+rdlen);
            while (bytes_to_parse.size() > 3) {
                // clear out our previous sample
                sample.clear();

                // try to parse a frame
                parse_result = parser.parse_frame(bytes_to_parse, sample);
                
                if (parse_result.bytes_used != 0) {
                    bytes_to_parse.erase(bytes_to_parse.begin(),bytes_to_parse.begin()+parse_result.bytes_used);
                }

                auto et = std::chrono::duration_cast<ms>(t1-t0).count();
    
                // I'm not sure I like using continue inside the switch
                switch (parse_result.status) {
                    case ParseResult::SUCCESS:

                        t1 = Time::now();
                        if ( et >= 100) {
                            fps = (float)frame_counter/((float)et/1000);
                            printf("got %.3f frames per sec\n", fps);
                            print_ops(&sample);
                            t0=t1;
                            frame_counter = 0;
                        } // if (et ..... timer for prints
                        frame_counter+=1;
                        continue;

                    case ParseResult::CONF_MISMATCH:
                    case ParseResult::BAD_START_BYTE:
                    case ParseResult::UNHANDLED_FRAME:
                        continue;
                    case ParseResult::NOT_ENOUGH_BYTES:
                    default:
                        break;
                } // switch
                break; // break the while loop if we get here
            } // while (bytes_to_parse.size() > 0)
        } // if (rdlen > 0)
        else if (rdlen < 0) {
            printf("Error from read: %d: %s\n", rdlen, strerror(errno));
        } else {  /* rdlen == 0 */
            printf("Timeout from read\n");
        }

    } while (1);
}
