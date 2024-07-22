#include "SFOCInterface.hpp"
#include "Parse.hpp"
#include "SimpleFOCRegisters.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>


/* define local struct for tty
 * load state into that struct
 * do stuff to it
 * set the state of the tty from the struct */
static int set_interface_attribs(int fd, int speed)
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

static void print_ops(Sample * const sample) {
    
    for (auto num : sample->operands) {
        
        uint8_t reg_type = SimpleFOCRegisters::regs->typeOfRegister(num.reg);
    
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

SFOC::SFOC(ParseType parse_type) : regs(SimpleFOCRegisters()) {
    (void) parse_type; // do something with this when relevant
    telem_conf = new TelemetryConfig(&regs);
    parser = new BinaryIOParser(&regs, *telem_conf);
}

int SFOC::run_serial(int serial_fd) {

    typedef std::chrono::high_resolution_clock Time;
    typedef std::chrono::milliseconds ms;
    auto t0 = Time::now();
    auto t1 = Time::now();


    // instance and manually set the telemetry configuration
    // TODO - ask the device for it's config and set this from that
    // ask for header frame 
    // send 2 sync frames

    telem_conf->num_operands = 5;
    telem_conf->operand_registers.push_back(SimpleFOCRegister::REG_TARGET);
    telem_conf->operand_registers.push_back(SimpleFOCRegister::REG_ANGLE);
    telem_conf->operand_registers.push_back(SimpleFOCRegister::REG_VELOCITY);
    telem_conf->operand_registers.push_back(SimpleFOCRegister::REG_CURRENT_Q);
    telem_conf->operand_registers.push_back(SimpleFOCRegister::REG_CURRENT_D);
    telem_conf->update_total();

    // setup some stuff to use in the main loop
    std::vector<uint8_t> bytes_to_parse;
    float fps=0; 
    size_t frame_counter = 0;
    Sample sample(telem_conf->num_operands);
    ParseResult parse_result;
    
    std::vector<std::unique_ptr<DataReq>> requests_to_service;
    // BinaryIOParser parser(&regs, telem_conf);

    std::vector<uint8_t> write_buf;
    write_buf.reserve(512);
    do {
        
        size_t samples_to_write = write_queue.get_length();
        // printf("have %lu samples to send!\n", samples_to_write);
        for (size_t i=0;i<samples_to_write;i++) {
            auto write_sample = write_queue.dequeue(); // shouldn't block because we just checked that there was something to dequeue
            auto bytes = parser->encode_frame(write_sample);
            if (bytes.size() != 0) {
                write_buf.insert(
                        write_buf.end(),
                        std::make_move_iterator(bytes.begin()),
                        std::make_move_iterator(bytes.end())
                        );
            }
        }
        // printf("encoded samples and wrote to send buffer\n");

        size_t write_len = 0;
        // printf("have %lu bytes to send!\n", write_buf.size());
        size_t bytes_to_write = write_buf.size();
        if (bytes_to_write > 0) {
            while (write_len != bytes_to_write) {
                // printf("starting serial write\n");
                write_len = write(serial_fd, &write_buf[0], write_buf.size());
                printf("wrote %lu bytes on the wire\n",write_len);
                if (write_len == write_buf.size()) {
                    write_buf.clear();
                    // printf("cleared write buffer\n");
                } else {
                    printf("didn't write all the bytes to the serial port!\n");
                    // printf("starting to remove bytes from buffer\n");

                    write_buf.erase(
                            write_buf.begin(),
                            write_buf.begin()+write_len);

                    // printf("removed bytes from buffer\n");
                }
            }
        }

        // get requests out of the incoming queue
        while (request_queue.get_length() > 0) {
            auto req = request_queue.try_dequeue();
            if (req) {
                requests_to_service.push_back(std::move(req));
            }
        }

        // printf("starting read from serial port\n");
        uint8_t read_buf[512];
        int rdlen;
        rdlen = read(serial_fd, read_buf, sizeof(read_buf) - 1);
        // printf("read from serial port\n");
        if (rdlen > 0) 
        { // we have some data to work with
            // put all the bytes from the read buffer into the end of the parse buffer
            bytes_to_parse.insert(bytes_to_parse.end(), read_buf, read_buf+rdlen);
            while (bytes_to_parse.size() > 3) {
                // clear out our previous sample
                sample.clear();

                // try to parse a frame
                parse_result = parser->parse_frame(bytes_to_parse, sample);
                
                if (parse_result.bytes_used != 0) {
                    // this is sloppy and inefficient because vectors aren't meant to be used this way
                    bytes_to_parse.erase(bytes_to_parse.begin(),bytes_to_parse.begin()+parse_result.bytes_used);
                }

                long long et; // = std::chrono::duration_cast<ms>(t1-t0).count();
    
                // I'm not sure I like using continue inside the switch
                switch (parse_result.status) {
                    case ParseResult::SUCCESS:

                        et = std::chrono::duration_cast<ms>(t1-t0).count();
                        t1 = Time::now();
                        if ( et >= 100) {
                            fps = (float)frame_counter/((float)et/1000);
                            // printf("got %.3f frames per sec\n", fps);
                            (void) fps;
                            // print_ops(&sample);
                            t0=t1;
                            frame_counter = 0;
                        } // if (et ..... timer for prints
                        frame_counter+=1;
                        service_requests(sample, requests_to_service);
                        // printf("servicing reqs\n");
                        // for (size_t i=0; i<requests_to_service.size(); i++) {
                        //     auto& req = requests_to_service[i];
                        //     if (req->n_samples < req->result_data->size()) {
                        //         req->result_data->push_back(sample);
                        //     } else { // req is fulfilled and can be returned
                        //         // idx_to_remove.push_back(i);
                        //         auto req_to_move = std::move(requests_to_service[i]);
                        //         data_queue.enqueue(std::move(req_to_move));
                        //     }
                        // }
                        // 
                        // // this can't be the right way to remove a set of stuff from a vector, can it?
                        // bool run = true;
                        // while (run) {
                        //     run=false;
                        //     for (size_t i=0; i<requests_to_service.size(); i++) {
                        //         if (!requests_to_service[i]) {
                        //             printf("removing req\n");
                        //             requests_to_service.erase(requests_to_service.begin()+i);
                        //             run=true;
                        //             break;
                        //         }
                        //     } // for
                        // } // while
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

    } while (run_thread);
    return 0;
}

int SFOC::connect(std::string port) {

    // open and configure the serial port
    int fd;
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", port.c_str(), strerror(errno));
        return -1;
    }

    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    // baud doesn't matter because using USB right now
    set_interface_attribs(fd, B115200);

    std::thread t1(&SFOC::run_serial, this, fd);

    active_threads.push_back(std::move(t1));
    
    return 0;
}

int SFOC::disconnect() {
    run_thread = false;
    for (auto &t : active_threads) {
        t.join();
    }
    active_threads.clear();
    return 0;
}

int SFOC::send_frame(Sample sample) {
    write_queue.enqueue(sample);
    return 0;
}

void SFOC::service_requests(Sample sample, std::vector<std::unique_ptr<DataReq>> &requests_to_service) {
    
    size_t sz = requests_to_service.size();
    for (size_t i=0; i<sz; i++) {
        auto& req = requests_to_service[i];
        if (req->n_samples > req->result_data->size()) {
            req->result_data->push_back(sample);
        } else { // req is fulfilled and can be returned
            // idx_to_remove.push_back(i);
            auto req_to_move = std::move(requests_to_service[i]);
            data_queue.enqueue(std::move(req_to_move));
        }
    }
    
    // this can't be the right way to remove a set of stuff from a vector, can it?
    bool run = true;
    while (run) {
        run=false;
        for (size_t i=0; i<requests_to_service.size(); i++) {
            if (!requests_to_service[i]) {
                printf("removing req\n");
                requests_to_service.erase(requests_to_service.begin()+i);
                run=true;
                break;
            }
        } // for
    } // while

}

uint64_t SFOC::req_data(size_t n_samples) {
    // construct a (shared/unique ptr?) DataReq
    // pass it by value to the req queue
    //
    // producer thread will remove it from the queue and add it to a list of reqs it's servicing
    // producer will continue servicing reqs until they're completed
    // producer will then pass the req and data back to main thread where it can be retrieved
    //
    // main thread will retrieve the data it's after from another function
    if (n_samples == 0) return 0;
    uint64_t req_handle = msg_id_cnt++; 
    auto request = std::make_unique<DataReq>(DataReq());
    request->req_type = DataReq::NUM;
    request->n_samples = n_samples;
    request->handle = req_handle; 
    request_queue.enqueue(std::move(request));
    return req_handle;
}

void SFOC::get_data_from_queue() {
    while (data_queue.get_length() > 0) {
        auto req = data_queue.try_dequeue();
        if (req) {
            completed_reqs.insert(std::make_pair(req->handle, std::move(req)));
        }
    }    
}

std::expected<std::unique_ptr<DataReq>, int> SFOC::get_req_data(uint64_t data_handle) {
    // this function will attempt to retrieve the data for the supplied request handle
    //
    // (in a function call)
    // first it removes all data in the producer return queue and puts it into 
    // a map with it's handle as the key
    //
    // then it uses the supplied handle to check if there's a matching record in 
    // the map. 
    //      if not matching handle, return unexpected error 
    //      if there is a matching handle, pop and return it
    if (data_handle==0) return std::unexpected(-1);
   
    get_data_from_queue();

    if (completed_reqs.find(data_handle) != completed_reqs.end()) {
        auto req = std::move(completed_reqs[data_handle]);
        completed_reqs.erase(data_handle);
        return req;
    }
    return std::unexpected(-2);
}
