// SPDX-License-Identifier: GPL-3.0-or-later
// Adapted from the tested OpenGarage ESPHome Sec+ 1.0 receiver/gap predictor.
#pragma once
#include <stdint.h>
#include <algorithm>
namespace SecPlus1Rx {
class Gap {
 public:
  // 1200 baud / 8E1 is about 9.17 ms per byte. Dequeue timestamps in
  // milliseconds can measure a valid adjacent query/reply as 9 or 10 ms.
  // This learning floor is separate from the unchanged 10 ms post-reply guard.
  static constexpr uint32_t MIN_REPLY_MS = 9;
#ifdef OG_SEC1_DIAGNOSTICS
  struct Diagnostic {
    uint32_t exchanges=0, resets=0, bad_latency=0, zero_latency=0, bad_interval=0;
    uint32_t latency=0, interval=0, previous_query=0;
    uint8_t max_count=0;
    bool have_query=false;
  } diagnostic;
  uint8_t learned_count() const { return count; }
  uint32_t learned_minimum() const { return minimum; }
  uint32_t learned_maximum() const { return maximum; }
  uint32_t reply_age(uint32_t now) const { return seen ? uint32_t(now-reply) : UINT32_MAX; }
#endif
  void reset() {
#ifdef OG_SEC1_DIAGNOSTICS
    ++diagnostic.resets;
#endif
    seen = false; count = 0;
  }
  void exchange(uint32_t q, uint32_t r) {
#ifdef OG_SEC1_DIAGNOSTICS
    ++diagnostic.exchanges;
    diagnostic.latency=uint32_t(r-q);
    if (diagnostic.latency<MIN_REPLY_MS || diagnostic.latency>60) ++diagnostic.bad_latency;
    if (!diagnostic.latency) ++diagnostic.zero_latency;
    if (diagnostic.have_query) {
      diagnostic.interval=uint32_t(q-diagnostic.previous_query);
      if(diagnostic.interval<180 || diagnostic.interval>400) ++diagnostic.bad_interval;
    }
    diagnostic.previous_query=q; diagnostic.have_query=true;
#endif
    if (uint32_t(r-q) < MIN_REPLY_MS || uint32_t(r-q) > 60) { reset(); return; }
    if (seen) {
      const uint32_t interval = q-query;
      if (interval < 180 || interval > 400 ||
          (count && (interval+20 < minimum || interval > maximum+20))) count=0;
      if (interval >= 180 && interval <= 400) {
        if (!count) minimum=maximum=interval;
        else { minimum=std::min(minimum,interval); maximum=std::max(maximum,interval); }
        if (maximum-minimum > 20) { minimum=maximum=interval; count=0; }
        if (count < 3) ++count;
      }
    }
    seen=true; query=q; reply=r;
#ifdef OG_SEC1_DIAGNOSTICS
    diagnostic.max_count=std::max(diagnostic.max_count,count);
#endif
  }
  bool ready(uint32_t now) const {
    const uint32_t age=now-query, reply_age=now-reply;
    return seen && count>=3 && reply_age>=10 && reply_age<=45 &&
        age<minimum && minimum-age>=160;
  }
  bool insufficient(uint32_t now) const {
    return seen && count>=3 && uint32_t(now-reply)<=400 && minimum<170U+uint32_t(reply-query);
  }
 private:
  uint32_t query=0, reply=0, minimum=0, maximum=0;
  uint8_t count=0;
  bool seen=false;
};
inline bool parity(uint8_t value) {
  value ^= value >> 4; value ^= value >> 2; value ^= value >> 1;
  return (value & 1) != 0;
}
struct Confirmed {
  uint8_t value=0, candidate=0, count=0;
  uint32_t confirmed_ms=0, candidate_ms=0;
  bool valid=false;
  void observe(uint8_t next, uint32_t now) {
    if (!count || next != candidate) { candidate=next; count=0; }
    candidate_ms=now;
    if (count<2) ++count;
    if (count==2) { value=next; valid=true; confirmed_ms=now; }
  }
  void tick(uint32_t now, uint32_t timeout) {
    if (count && uint32_t(now-candidate_ms)>=timeout) count=0;
    if (valid && uint32_t(now-confirmed_ms)>=timeout) { valid=false; count=0; }
  }
};
class Receiver {
 public:
  // Each field is polled about every 750 ms in emulation. 15 s tolerates
  // initialization/ordinary missed exchanges without retaining status forever.
  static constexpr uint32_t TIMEOUT=15000;
  Confirmed door, light_lock;
  Gap gap;
  bool obstruction=false, obstruction_valid=false, observed=false, panel37=false;
  uint32_t last_byte=0, obstruction_ms=0, parity_errors=0, invalid_frames=0, frames=0, overflows=0;
  uint8_t request=0;
  uint32_t query_ms=0;
  void tick(uint32_t now) {
    if (request && uint32_t(now-last_byte)>=100) { request=0; gap.reset(); invalidate_candidates(); }
    door.tick(now,TIMEOUT); light_lock.tick(now,TIMEOUT);
    if (obstruction_valid && uint32_t(now-obstruction_ms)>=TIMEOUT) obstruction_valid=false;
  }
  void invalidate_candidates() { door.count=light_lock.count=0; }
  void loss() {
    request=0; gap.reset(); invalidate_candidates();
    door.valid=light_lock.valid=obstruction_valid=false;
  }
  void start(uint8_t byte) {
    if (byte==0x38 || byte==0x39 || byte==0x3a) { request=byte; query_ms=last_byte; }
    else { gap.reset(); if (byte==0x37) panel37=true; }
  }
  void feed(uint8_t byte, bool parity_bit, uint32_t now) {
    tick(now); last_byte=now;
    if (parity_bit != parity(byte)) { ++parity_errors; request=0; gap.reset(); invalidate_candidates(); return; }
    if (!request) { start(byte); return; }
    const uint8_t command=request; request=0;
    // A command/release echo is not a status payload, notably 0x31 != Opening.
    if (byte>=0x30 && byte<=0x3a) {
      ++invalid_frames; gap.reset(); invalidate_candidates(); start(byte); return;
    }
    if (command==0x38) {
      const uint8_t value=byte & 7;
      if (value==3 || value==7) { ++invalid_frames; gap.reset(); door.valid=false; door.count=0; return; }
      door.observe(value==6 ? 0 : value,now); observed=true;
    } else if (command==0x3a) {
      if ((byte & 0xf0)!=0x50) { ++invalid_frames; gap.reset(); light_lock.valid=false; light_lock.count=0; return; }
      light_lock.observe(byte & 12,now); observed=true;
    } else {
      obstruction=byte!=0; obstruction_valid=true; obstruction_ms=now;
    }
    ++frames;
    gap.exchange(query_ms,now);
  }
};
}  // namespace SecPlus1Rx
