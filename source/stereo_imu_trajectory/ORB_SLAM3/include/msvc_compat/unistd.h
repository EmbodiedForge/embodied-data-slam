#pragma once

#include <chrono>
#include <thread>

inline int usleep(unsigned int usec) {
    std::this_thread::sleep_for(std::chrono::microseconds(usec));
    return 0;
}
