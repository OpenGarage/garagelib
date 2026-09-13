#include <Arduino.h>
#include <SoftwareSerial.h>
#include <secplus.h>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include "../secplus2_recovery.h"
#define private public
#include "../garagelib.cpp"
#undef private
using namespace SecPlus2Recovery;
static void status(SecPlus2::Garage &g,uint32_t id=0x12342908) {
  uint8_t packet[19];
  assert(encode_wireline(100,id,0x81 | (2<<8),packet)==0);
  assert(g.process_packet(packet)==0);
}
static void policy() {
  Session s; uint32_t now=1000; s.start(now);
  for (auto wait : {30000U,60000U,120000U,240000U,300000U,300000U}) {
    assert(s.due(now)==Query::STATUS); s.sent(now,true);
    assert(s.due(now+99)==Query::NONE);
    assert(s.due(now+100)==Query::OPENINGS); s.sent(now+100,true);
    assert(s.due(now+200)==Query::NONE);
    s.tick(now+5000); assert(s.phase==Session::Phase::BACKOFF);
    s.status(now+5001); s.tick(now+5001); assert(!s.ready(now+5001));
    s.tick(now+5000+wait-1); assert(s.phase==Session::Phase::BACKOFF);
    now+=5000+wait; s.tick(now); assert(s.phase==Session::Phase::SEEKING);
  }
  s.sent(now,true); s.status(now+1); s.tick(now+1); assert(!s.ready(now+1));
  s.sent(now+100,true); s.tick(now+101); assert(s.ready(now+101));
  assert(s.due(now+10100)==Query::NONE);
  assert(s.due(now+10101)==Query::STATUS);
  s.sent(now+10101,false); assert(s.phase==Session::Phase::BACKOFF);
  s.tick(now+40101); assert(s.phase==Session::Phase::SEEKING); // successful observation reset backoff
  Session wrap; const uint32_t start=UINT32_MAX-50; wrap.start(start); wrap.sent(start,true);
  assert(wrap.due(uint32_t(start+100))==Query::OPENINGS);
  wrap.sent(uint32_t(start+100),true); wrap.status(uint32_t(start+101)); wrap.tick(uint32_t(start+101));
  assert(wrap.ready(uint32_t(start+101)));
  wrap.tick(uint32_t(start+30101)); assert(!wrap.ready(uint32_t(start+30101)));
}
static void integrated() {
  SecPlus2::Garage g(0x56782908,5,15);
  test_now=1000; g.begin(); g.reset_state();
  auto *uart=SecPlusCommon::sw_serial;
  g.loop(); assert(uart->tx.size()==19);
  test_now=1100; g.loop(); assert(uart->tx.size()==38);
  status(g); g.loop(); assert(g.recovery.ready(test_now));
  // Own RX echo neither supplies freshness nor double-applies a light toggle.
  uint8_t light[19];
  assert(encode_wireline(100,(uint64_t(0x200)<<24)|0x56782908,0x81|(2<<8),light)==0);
  const bool before=g.state.light_state;
  g.process_packet(light); assert(g.state.light_state==before);
  g.process_packet(light,false); assert(g.state.light_state!=before);
  g.process_packet(light); assert(g.state.light_state!=before);
  // A press colliding before transmission expires instead of replaying later.
  assert(g.open_door()==0); test_rx_high=1; test_now=1200; g.loop();
  assert(uart->tx.size()==38);
  test_now=3100; g.loop(); assert(g.buf.get_size()==0 && g.expired_commands==1);
  test_rx_high=0; test_now=3200; g.loop(); assert(uart->tx.size()==38);
  // Uncertain press delivery still releases; controls fault, status queries continue.
  assert(g.open_door()==0); uart->short_write=true; g.loop();
  assert(g.release_pending && g.control_fault && g.buf.get_size()==1);
  uart->short_write=false; test_now=3450; g.loop(); assert(!g.release_pending && g.buf.get_size()==0);
  assert(g.close_door()<0);
  size_t count=uart->tx.size(); test_now=11100; g.loop(); assert(uart->tx.size()==count+19);
  // Opener remains absent >90 s while OG keeps its counter and does bounded retries.
  g.reset_state(); uart->tx.clear(); test_now=12000; g.loop(); test_now=12100; g.loop();
  const uint32_t after_pair=g.rolling_code;
  test_now=16100; g.loop(); assert(g.recovery.phase==Session::Phase::BACKOFF);
  assert(!g.status_valid() && g.get_door_state()==SecPlusCommon::DoorStatus::UNKNOWN);
  test_now=46100; g.loop(); test_now=46200; g.loop();
  assert(g.rolling_code==after_pair+2);
  test_now=51100; g.loop();
  status(g); g.loop(); assert(!g.recovery.ready(test_now)); // unsolicited frame cannot bypass paced retry
  test_now=111100; g.loop(); test_now=111200; g.loop(); status(g); g.loop();
  assert(g.recovery.ready(test_now)); assert(g.toggle_light()==0);
  test_now+=100; g.loop();
  // A failed query heals through the same retry path, not a reboot-only latch.
  test_now+=10000; uart->short_write=true; g.loop();
  assert(g.query_failures>0 && !g.control_fault && g.recovery.phase==Session::Phase::BACKOFF);
  uart->short_write=false; test_now+=30000; g.loop(); test_now+=100; g.loop(); status(g); g.loop();
  assert(g.recovery.ready(test_now));
  g.stop(); assert(g.toggle_door()<0);
  SecPlusCommon::delete_sw_serial();
}
int main() { policy(); integrated(); std::puts("Sec+ 2.0 recovery: policy, outage, collisions, partial writes, self echo and no-replay tests passed"); }
