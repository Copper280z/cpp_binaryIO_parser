#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class ThreadSafeQueue {
private:
    std::queue<T> q;
    std::mutex mtx;
    std::condition_variable cv;

public:
    void enqueue(const T& item) {
        std::lock_guard<std::mutex> lock(mtx);
        q.push(item);
        cv.notify_one(); // Notify waiting threads
    }

    T dequeue() {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return !q.empty(); }); // Wait until queue is not empty
        T frontItem = q.front();
        q.pop();
        return frontItem;
    }
};

