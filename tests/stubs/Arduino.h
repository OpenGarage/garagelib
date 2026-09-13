#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
using uint16 = uint16_t;
#define HIGH 1
#define LOW 0
inline uint32_t test_now = 1;
inline int test_rx_high = 0;
inline uint32_t millis() { return test_now; }
inline void yield() {}
inline void delayMicroseconds(unsigned) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return test_rx_high; }
