#pragma once
#include <Arduino.h>
#include <vector>
#include <deque>
#define SWSERIAL_8N1 0
#define SWSERIAL_8E1 1
class SoftwareSerial {
 public:
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;
  bool parity_bit=false, short_write=false;
  void begin(int, int, int, int, bool) {}
  void end() {}
  void enableIntTx(bool) {}
  int available() { return rx.size(); }
  int read() { if (rx.empty()) return -1; auto b=rx.front(); rx.pop_front(); return b; }
  bool readParity() { return parity_bit; }
  bool overflow() { return false; }
  size_t write(const uint8_t *p, size_t size) {
    tx.insert(tx.end(),p,p+size);
    return short_write ? 0 : size;
  }
  size_t write(uint8_t b) { return write(&b,1); }
};
