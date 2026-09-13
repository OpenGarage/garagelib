#include <Arduino.h>
#include <SoftwareSerial.h>
#include <secplus.h>
#include <cassert>
#include <cstdio>
#include <initializer_list>
#ifndef GARAGELIB_SOURCE
#define GARAGELIB_SOURCE "../garagelib.cpp"
#endif
// Test the real decoder without adding a test-only production API.
#define private public
#include GARAGELIB_SOURCE
#undef private

int main() {
  SecPlus2::Garage disabled(0, 4, 5);
  disabled.begin();
  assert(SecPlusCommon::sw_serial == nullptr);
  assert(!disabled.detect());
  disabled.loop();
  assert(disabled.request_status() < 0);
  assert(disabled.toggle_light() < 0);
  assert(disabled.toggle_lock() < 0);
  assert(disabled.toggle_door() < 0);
  assert(disabled.buf.get_size() == 0);
  assert(!disabled.set_client_id(0));
  assert(disabled.set_client_id(0x12342908));
  assert(disabled.request_status() == 0);
  disabled.begin(); disabled.reset_state(); disabled.loop();
  uint32_t rolling, data;
  uint64_t fixed;
  assert(decode_wireline(SecPlusCommon::sw_serial->tx.data(), &rolling, &fixed, &data) == 0);
  assert(uint32_t(fixed) == 0x12342908);
  assert(!disabled.set_client_id(0x56782908));
  SecPlusCommon::delete_sw_serial();
  unsigned checks = 0;
  for (auto command : {SecPlus2::Command::LIGHT, SecPlus2::Command::LOCK}) {
    for (unsigned before = 0; before < 2; ++before) {
      for (unsigned value = 0; value < 4; ++value) {
        SecPlus2::Garage garage(0x2908, 4, 5);
        garage.state.light_state = before;
        garage.state.lock_state = before;
        uint8_t packet[19];
        const uint16_t cmd = static_cast<uint16_t>(command);
        const uint64_t fixed = (uint64_t(cmd & 0xf00) << 24) | 0x12342908;
        assert(encode_wireline(0, fixed, (value << 8) | (cmd & 0xff), packet) == 0);
        assert(garage.process_packet(packet) == 0);
        const bool expected = value == 0 ? false : value == 1 ? true :
                              value == 2 ? !before : bool(before);
        assert(garage.state.light_state == (command == SecPlus2::Command::LIGHT ? expected : bool(before)));
        assert(garage.state.lock_state == (command == SecPlus2::Command::LOCK ? expected : bool(before)));
        ++checks;
      }
    }
  }
  std::printf("Sec+ 2.0 LIGHT/LOCK: %u encoded-frame cases passed\n", checks);
}
