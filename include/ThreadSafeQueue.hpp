#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <utility>

template <typename T>
class ThreadSafeQueue {
private:
    std::queue<T> q;
    std::mutex mtx;
    std::condition_variable cv;

public:
    // void enqueue(const T& item) {
    //     std::lock_guard<std::mutex> lock(mtx);
    //     q.push(item);
    //     cv.notify_one(); // Notify waiting threads
    // }

    void enqueue(T item) {
        std::lock_guard<std::mutex> lock(mtx);
        q.push(std::move(item));
        cv.notify_one(); // Notify waiting threads
    }

    T dequeue() {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return !q.empty(); }); // Wait until queue is not empty
        T frontItem = std::move(q.front());
        q.pop();
        return frontItem;
    }
    
    T try_dequeue() {
        std::unique_lock<std::mutex> lock(mtx);
        if (q.empty()) {
            return T();
        } 

        cv.wait(lock, [this] { return !q.empty(); }); // Wait until queue is not empty
        T frontItem = std::move(q.front());
        q.pop();
        return frontItem;
    }

    size_t get_length() {
        std::unique_lock<std::mutex> lock(mtx);
        return q.size();
    }
};

