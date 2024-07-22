#include <chrono>
#include <cstddef>
#include <vector>

#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>



#include "SFOCInterface.hpp"
#include "SimpleFOCRegisters.hpp"

#define TERMINAL    "/dev/ttyACM0"

using namespace std;
using namespace std::chrono_literals;

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

// sync len  type  id   [1   2   3   4]   [1    2    3    4]    [1    2    3    4]    [1    2    3    4]    [1    2    3    4]
// 0xa5 0x16 0x54 0x0   0x0 0x0 0x0 0x0   0xd9 0xa0 0xd5 0xbc   0x11 0x99 0x10 0xc0   0xb4 0x9a 0x81 0x3c   0x52 0xa1 0x11 0x3c
// 0xa5 0x16 0x54 0x0   0x0 0x0 0x0 0x0   0x6a 0xfd 0xba 0x3e   0x2b 0xe4 0xe4 0x41   0xb1 0x3c 0xbd 0xbe   0x31 0x77 0x62 0xbc
// 0xa5 0x16 0x54 0x0   0x0 0x0 0x0 0x0   0x80 0xdb 0xbb 0x3e   0x17 0x99 0xe1 0x41   0x40 0x47 0xb6 0xbe   0x5c 0xc4 0x5c 0xbb

SFOC sfoc(ParseType::Binary);

int main(int argc, char **argv)
{
    std::string portname(TERMINAL);

    InputParser input(argc, argv);
    if(input.cmdOptionExists("-h")){
        printf("Usage: \n");
        printf("-p SERIAL_PORT - ie /dev/ttyACM0 or /dev/cu.usbmodem[...]\n");
    }
    const std::string &port = input.getCmdOption("-p");
    if (!port.empty()){
        portname = port.c_str();
    }

    sfoc.connect(portname.c_str());

    Sample frame1(1);
    frame1.frame_type = FrameType::REGISTER;
    frame1.num=0;
    frame1.address = SimpleFOCRegister::REG_TELEMETRY_REG;

    sfoc.send_frame(frame1);
    frame1.address=0; 
    frame1.num=1;
    Number val;
    val.reg = SimpleFOCRegister::REG_TARGET;
    val.type = RegType::FLOAT;
    val.f = 3.14;

    frame1.operands.push_back(val);

    sfoc.send_frame(frame1);

    std::this_thread::sleep_for(400ms);

    frame1.operands[0].f = 0.0;
    sfoc.send_frame(frame1);
    
    std::this_thread::sleep_for(500ms);
    
    sfoc.disconnect();
}
