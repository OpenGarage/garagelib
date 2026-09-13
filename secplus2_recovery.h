// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
namespace SecPlus2Recovery {
enum class Query : uint8_t { NONE, STATUS, OPENINGS };
class Session {
 public:
  static constexpr uint32_t STATUS_TIMEOUT=30000, POLL_INTERVAL=10000;
  static constexpr uint32_t SEEK_WINDOW=5000, PAIR_SPACING=100;
  enum class Phase { SEEKING, BACKOFF, OBSERVED };
  void start(uint32_t now) {
    phase=Phase::SEEKING; started=now; attempts=0; backoff=30000;
    status_seen=false; last_status=last_attempt=now; sequence=baseline=0;
  }
  void status(uint32_t now) { status_seen=true; last_status=now; ++sequence; }
  bool fresh(uint32_t now) const { return status_seen && uint32_t(now-last_status)<STATUS_TIMEOUT; }
  bool ready(uint32_t now) const { return phase==Phase::OBSERVED && fresh(now); }
  void tick(uint32_t now) {
    if (phase==Phase::OBSERVED && !fresh(now)) seek(now);
    if (phase==Phase::BACKOFF && uint32_t(now-started)>=wait) seek(now);
    if (phase==Phase::SEEKING) {
      // RX alone never recovers BACKOFF. Complete our paced pair first.
      if (attempts==2 && fresh(now) && sequence!=baseline) {
        phase=Phase::OBSERVED; backoff=30000; last_attempt=now;
      } else if (uint32_t(now-started)>=SEEK_WINDOW) failed(now);
    }
  }
  Query due(uint32_t now) const {
    if (phase==Phase::SEEKING) {
      if (!attempts) return Query::STATUS;
      if (attempts==1 && uint32_t(now-last_attempt)>=PAIR_SPACING) return Query::OPENINGS;
    } else if (phase==Phase::OBSERVED && uint32_t(now-last_attempt)>=POLL_INTERVAL) return Query::STATUS;
    return Query::NONE;
  }
  void sent(uint32_t now, bool ok) {
    if (!ok) { failed(now); return; }
    if (phase==Phase::SEEKING) {
      if (!attempts) baseline=sequence;
      if (attempts<2) ++attempts;
    }
    last_attempt=now;
  }
  void request_status(uint32_t now) {
    if (phase==Phase::OBSERVED) last_attempt=now-POLL_INTERVAL;
  }
  const char *description() const {
    if (phase==Phase::BACKOFF) return "No query response; automatic retry scheduled";
    if (phase==Phase::SEEKING) return "Querying opener; responses are not TX acknowledgements";
    return "Status observed (not TX acknowledgement)";
  }
  Phase phase=Phase::SEEKING;
 private:
  void seek(uint32_t now) { phase=Phase::SEEKING; started=now; attempts=0; baseline=sequence; }
  void failed(uint32_t now) {
    phase=Phase::BACKOFF; started=now; wait=backoff;
    backoff=backoff>=150000 ? 300000 : backoff*2;
  }
  uint32_t started=0, last_status=0, last_attempt=0, backoff=30000, wait=30000;
  uint32_t sequence=0, baseline=0;
  uint8_t attempts=0;
  bool status_seen=false;
};
}
