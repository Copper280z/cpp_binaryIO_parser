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

SFOC sfoc(ParseType::Binary);

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

    sfoc.connect(portname.c_str());

    std::this_thread::sleep_for(10000ms);
    
    sfoc.disconnect();
}
