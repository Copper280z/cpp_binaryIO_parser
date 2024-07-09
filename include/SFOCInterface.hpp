#pragma once

#include "Parse.hpp"
#include "SimpleFOCRegisters.hpp"
#include "ThreadSafeQueue.hpp"
#include <vector>
#include <thread>

typedef enum {
    Binary,
    ASCII
} ParseType; 

class SFOC {
    public:
        SFOC(ParseType parse_type);
    
        // get next telemetry frame
        Sample get_next_frame();

        // get N frames
        std::vector<Sample> get_frames(size_t num_frames, bool block);

        // connect
        int connect(std::string port);

        // disconnect
        int disconnect();

        int send_frame(Sample sample);

    private:
        // opaque pointer?
        ThreadSafeQueue<std::vector<Sample>> data_queue;
        ThreadSafeQueue<Sample> write_queue;
        std::string _port_name; 
        SimpleFOCRegisters regs;
        TelemetryConfig * telem_conf; // maybe this should live in the parser?
        BinaryIOParser * parser; // create parser base class and replace here

        std::vector<std::thread> active_threads;
        std::atomic<bool> run_thread = true;
        int run_serial(int serial_fd);
};
