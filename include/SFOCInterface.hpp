#pragma once

#include "Parse.hpp"
#include "SimpleFOCRegisters.hpp"
#include "ThreadSafeQueue.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <vector>
#include <thread>
#include <map>

typedef enum {
    Binary,
    ASCII
} ParseType; 

class DataReq {
    public:
        enum mode {NUM, TIME};
        mode req_type = mode::NUM;
        size_t n_samples = 0;
        size_t timeout_msec = 0;
        size_t bad_packets = 0;
        
        uint64_t handle = 0;

        std::unique_ptr<std::vector<Sample>> result_data;
        DataReq() { result_data = std::make_unique<std::vector<Sample>>(std::vector<Sample>{});};
};

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
        
        uint64_t req_data(size_t n_samples);
        std::expected<std::unique_ptr<DataReq>, int> get_req_data(uint64_t data_handle);

    private:
        // opaque pointer?
        
        ThreadSafeQueue<std::unique_ptr<DataReq>> request_queue;
        ThreadSafeQueue<std::unique_ptr<DataReq>> data_queue;
        ThreadSafeQueue<Sample> write_queue;

        std::map<uint64_t, std::unique_ptr<DataReq>> completed_reqs;

        std::string _port_name; 
        SimpleFOCRegisters regs;
        TelemetryConfig * telem_conf; // maybe this should live in the parser?
        BinaryIOParser * parser; // create parser base class and replace here

        std::vector<std::thread> active_threads;
        std::atomic<bool> run_thread = true;
        int run_serial(int serial_fd);
    
        void get_data_from_queue();

        void service_requests(Sample sample, std::vector<std::unique_ptr<DataReq>> &requests_to_service);
        // void service_requests(Sample sample);

        // use an integer as the id for each request passed to the thread
        // each message has and id, and you retrieve the message with that id
        // once it's fulfilled
        std::atomic<uint64_t> msg_id_cnt = 1;
};
